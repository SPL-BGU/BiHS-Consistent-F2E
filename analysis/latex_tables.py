import math
import pathlib
from typing import Union, List

import pandas as pd

BOUND_NUMBERS = {'fd': 1,
                 'df': 2,
                 'g': 3,
                 'b': 4,
                 'gfgd': 5,
                 'gdgf': 6,
                 'gb': 7,
                 'rfrd': 8,
                 'rdrf': 9,
                 'frdrfd': 10,
                 'rfdfrd': 11,
                 'grfgrd': 12,
                 'grdgrf': 13,
                 'rfrdx2': 14,
                 'gfrdgrfd': 15,
                 'grfdgfrd': 16,
                 'grfrd': 17}


def alg_key(algname: str):
    if algname == 'A*':
        return 1
    if algname == 'rA*':
        return 2
    if algname == 'NBS':
        return 3
    if algname == 'DVCBS':
        return 3.5
    if algname.startswith('MVC'):
        if algname == 'MVC(GMX)':
            return 4
        if algname.startswith('MVC(\\'):
            return 5
        return 4.5
    if algname.startswith('BAE'):
        if algname.endswith('a)'):
            return 5.5
        if algname.endswith('p)'):
            return 5.6
        return 5.7
    if algname.startswith('\\TB'):
        prio = 6
        algname = algname.split('{')
        prio += int(algname[1][:-1]) / 100
        side = algname[2][:-1]
        if side == 'a':
            return prio + 0.0001
        if side == 'p':
            return prio + 0.0002
        return prio + 0.0003

    prio = 8 if algname.startswith('\\DBBSF') else 7
    algname = algname.split('{')
    priority = algname[1][:-1]
    if priority == 'b':
        prio += 0.1
    elif priority == 'ss':
        prio += 0.2
    else:
        prio += 0.3
    side = algname[2][:-1]
    if side == 'a':
        return prio + 0.0001
    if side == 'p':
        return prio + 0.0002
    return prio + 0.0003


def alg_key_series(series: pd.Series) -> pd.Series:
    return series.apply(alg_key)


def normal_round(n):
    if math.isnan(n):
        return n
    if n - math.floor(n) < 0.5:
        return math.floor(n)
    return math.ceil(n)


def format_expansions(num):
    num = normal_round(num)
    if num < 10000:
        return "{:,}".format(num)  # Add thousand separator
    else:
        return "{:,}K".format(normal_round(num / 1000))  # Convert to K and add separator


def format_alg_name(algname: str):
    if algname in ['A*', 'DVCBS', 'NBS']:
        return algname
    if algname == 'RA*':
        return 'rA*'
    if algname == 'GMX':
        return 'MVC(\\GMXCU)'
    if algname == 'GMXC':
        return 'MVC(GMX$_\\text{C}$)'
    if algname == 'NGMX':
        return 'MVC(GMX)'
    if algname == 'BAE-FBI':
        return 'BAE*(FBI)'

    if algname.startswith('BAE*') or algname.startswith('tb') or algname.startswith('TB'):
        algname = algname.split('-')
        if algname[1] == 'o':
            return f'BAE*({algname[2]})'
        return f'\\TB{{{BOUND_NUMBERS[algname[1]]}}}{{{"FBI" if algname[2] == "fbi" else algname[2]}}}'

    if algname.startswith('DBBS'):
        algname = algname.split('-')
        algorithm = '\\DBBSF' if algname[0] in ['DBBSLB', 'DBBSF'] else '\\DBBS'
        side = algname[1] if algname[1] != 'o' else 'FBI'
        if side == 'fbi':
            side = 'FBI'
        priority = 'b' if len(algname) == 2 else algname[2]
        priority = priority if priority != 'MaxTot' else 'max'
        priority = priority if priority != 'MinTot' else 'ss'
        return f'{algorithm}{{{priority}}}{{{side}}}'

    raise ValueError(f'Unhandled Algorithm: {algname}')


def reverse_format_expansions(n_str: str):
    if n_str.endswith('K'):
        return int(n_str[:-1].replace(',', '')) * 1000
    else:
        return int(n_str.replace(',', ''))


def single_heuristic_table(df, instance_num=100):
    cdf = df[~df['alg'].str.contains('GMX')]
    necessary_min = cdf['necessary_avg'].apply(reverse_format_expansions).idxmin()
    necessary_min = cdf['necessary_avg'][necessary_min]
    expanded_min = cdf['expanded_avg'].apply(reverse_format_expansions).idxmin()
    expanded_min = cdf['expanded_avg'][expanded_min]
    table_text = '\\begin{tabular}{|l|r|r|}\n'
    table_text += '\\hline\n'
    table_text += '\\multicolumn{1}{|c|}{\\textbf{Algorithm}} & \\multicolumn{1}{c|}{\\textbf{\\totalExp}} & \\multicolumn{1}{c|}{\\textbf{\\necessaryExp}}\\\\\n'
    table_text += '\\hline\n'
    for index, row in df.iterrows():
        table_text += f'{row["alg"]} & '
        if row['instance_count'] != instance_num or instance_num < 0:
            table_text += f'\\multicolumn{{2}}{{c|}}{{\\#{row["instance_count"]}}}\\\\\n'
        else:
            expanded = '' if 'GMX' in row['alg'] else row["expanded_avg"]
            if expanded == expanded_min:
                table_text += f'\\textbf{{{expanded}}}'
            else:
                table_text += f'{expanded}'
            table_text += ' & '
            if row["necessary_avg"] == necessary_min and 'GMX' not in row['alg']:
                table_text += f'\\textbf{{{row["necessary_avg"]}}}'
            else:
                table_text += f'{row["necessary_avg"]}'
            table_text += f' \\\\\n'
    table_text += '\\hline\n'
    table_text += '\\end{tabular}\n'
    return table_text


def multi_heuristic_table(df, heuristics, instance_num: Union[int, List[int]] = 50):
    if type(instance_num) == int:
        instance_num = [instance_num] * len(heuristics)
    min_map = {}
    for icount, heuristic in zip(instance_num, heuristics):
        cdf = df[(~df['alg'].str.contains('GMX')) & (df[f'{heuristic}_instance_count'] == icount)]
        necessary_min_id = cdf[f"{heuristic}_necessary_avg"].apply(reverse_format_expansions).idxmin()
        min_map[f"{heuristic}_necessary_avg"] = cdf[f"{heuristic}_necessary_avg"][necessary_min_id]
        expanded_min_id = cdf[f"{heuristic}_expanded_avg"].apply(reverse_format_expansions).idxmin()
        min_map[f"{heuristic}_expanded_avg"] = cdf[f"{heuristic}_expanded_avg"][expanded_min_id]
    table_text = '\\begin{tabular}{|l|' + ('r|r|' * len(heuristics)) + '}\n'
    table_text += '\\cline{2-' + str(len(heuristics) * 2 + 1) + '}\n'
    table_text += '\\multicolumn{1}{c|}{}'
    for heuristic in heuristics:
        table_text += ' & \multicolumn{2}{c|}{\\textbf{' + heuristic + '}}'
    table_text += '\\\\ \\hline\n'
    table_text += '\\multicolumn{1}{|c|}{\\textbf{Algorithm}}'
    for _ in heuristics:
        table_text += ' & \multicolumn{1}{c|}{\\textbf{\\totalExp}} & \\multicolumn{1}{c|}{\\textbf{\\necessaryExp}}'
    table_text += '\\\\ \\hline\n'
    for index, row in df.iterrows():
        table_text += row['alg']
        for icount, heuristic in zip(instance_num, heuristics):
            instance_count = row[f'{heuristic}_instance_count']
            if instance_count != icount or icount < 0:
                if math.isnan(instance_count):
                    instance_count = 'N{\\textbackslash}A'
                else:
                    instance_count = f'\\#{instance_count}'
                table_text += f' & \\multicolumn{{2}}{{c|}}{{{instance_count}}}'
            else:
                table_text += ' & '
                expanded = '' if 'GMX' in row['alg'] else row[f"{heuristic}_expanded_avg"]
                necessary = row[f"{heuristic}_necessary_avg"]
                if expanded == min_map[f"{heuristic}_expanded_avg"]:
                    table_text += f'\\textbf{{{expanded}}}'
                else:
                    table_text += f'{expanded}'
                table_text += ' & '
                if necessary == min_map[f"{heuristic}_necessary_avg"]:
                    table_text += f'\\textbf{{{necessary}}}'
                else:
                    table_text += f'{necessary}'
        table_text += ' \\\\\n'
    table_text += '\\hline\n'
    table_text += '\\end{tabular}\n'
    return table_text


def load_csvs(file_paths: List[Union[pathlib.Path, str]]):
    dataframes = [pd.read_csv(path) for path in file_paths]

    # Verify all dfs have the same columns
    for i, df in enumerate(dataframes):
        df.rename(columns={'expansions': 'expanded'}, inplace=True)
        if (df.columns != dataframes[0].columns).any():
            raise ValueError(f"CSV at {file_paths[i]} does not have the same columns.")

    return pd.concat(dataframes, ignore_index=True)


def generate_stp_table(file_paths: List[Union[pathlib.Path, str]]):
    df = load_csvs(file_paths)
    df = df[
        (df['heuristic'] != 'MD-5') & (df['alg'] != 'BTB-conn') & (df['alg'] != 'BTB-FBI') & (df['alg'] != 'BTB-nbs')]
    df = df[~df['alg'].str.startswith('CSBS')]

    df = df.groupby('alg').agg(
        expanded_avg=('expanded', 'mean'),
        necessary_avg=('necessary', 'mean'),
        instance_count=('instance', 'count')
    ).reset_index()

    df['alg'] = df['alg'].apply(format_alg_name)
    df['expanded_avg'] = df['expanded_avg'].apply(format_expansions)
    df['necessary_avg'] = df['necessary_avg'].apply(format_expansions)

    df = df.sort_values(by='alg', key=alg_key_series)
    return df


def generate_road_table(file_paths: List[Union[pathlib.Path, str]]):
    df = load_csvs(file_paths)
    df = df[~df['alg'].str.startswith('CSBS')]
    df = df.groupby('alg').agg(
        expanded_avg=('expanded', 'mean'),
        necessary_avg=('necessary', 'mean'),
        instance_count=('instance', 'count')
    ).reset_index()

    df['alg'] = df['alg'].apply(format_alg_name)
    df['expanded_avg'] = df['expanded_avg'].apply(format_expansions)
    df['necessary_avg'] = df['necessary_avg'].apply(format_expansions)

    df = df.sort_values(by='alg', key=alg_key_series)
    return df


def generate_dao_table(file_paths: List[Union[pathlib.Path, str]]):
    df = load_csvs(file_paths)
    df = df[~df['alg'].str.startswith('CSBS')]
    df = df[(df['alg'] != 'BTB-conn') & (df['alg'] != 'BTB-FBI') & (df['alg'] != 'BTB-nbs')]

    df = df.groupby('alg').agg(
        expanded_avg=('expanded', 'mean'),
        necessary_avg=('necessary', 'mean'),
        instance_count=('instance', 'count')
    ).reset_index()

    def dao_format_expansions(expansions):
        return "{:,}".format(normal_round(expansions))

    df['alg'] = df['alg'].apply(format_alg_name)
    df['expanded_avg'] = df['expanded_avg'].apply(dao_format_expansions)
    df['necessary_avg'] = df['necessary_avg'].apply(dao_format_expansions)
    df = df.sort_values(by='alg', key=alg_key_series)

    return df


def generate_toh_table(file_paths: List[Union[pathlib.Path, str]]):
    df = load_csvs(file_paths)
    df = df[(df['alg'] != 'BTB-conn') & (df['alg'] != 'BTB-FBI') & (df['alg'] != 'BTB-nbs')]

    df['heuristic'] = df['heuristic'].replace({
        '(10+2)': 'PDB-2',
        '(8+4)': 'PDB-4',
        '(6+6)': 'PDB-6'
    })

    df = df.groupby(by=['heuristic', 'alg']).agg(
        expanded_avg=('expanded', 'mean'),
        necessary_avg=('necessary', 'mean'),
        instance_count=('instance', 'count')
    ).reset_index()

    df['alg'] = df['alg'].apply(format_alg_name)
    df['expanded_avg'] = df['expanded_avg'].apply(format_expansions)
    df['necessary_avg'] = df['necessary_avg'].apply(format_expansions)

    df = df.sort_values(by='alg', key=alg_key_series)
    df_pivot = df.pivot(index='alg', columns='heuristic', values=['expanded_avg', 'necessary_avg', 'instance_count'])

    # Flatten the column multi-index and place 'PDB' heuristic first, followed by metric
    df_pivot.columns = [f"{heuristic}_{metric}" for metric, heuristic in df_pivot.columns]

    # Reset the index for a clean look
    df_pivot = df_pivot.reset_index()
    df_pivot = df_pivot.sort_values(by='alg', key=alg_key_series)

    return df_pivot


def generate_pancake_table(file_paths: List[Union[pathlib.Path, str]]):
    df = load_csvs(file_paths)
    df = df[(df['alg'] != 'BTB-conn') & (df['alg'] != 'BTB-FBI') & (df['alg'] != 'BTB-nbs')]

    df = df.groupby(by=['heuristic', 'alg']).agg(
        expanded_avg=('expanded', 'mean'),
        necessary_avg=('necessary', 'mean'),
        instance_count=('instance', 'count')
    ).reset_index()

    df['alg'] = df['alg'].apply(format_alg_name)
    df['expanded_avg'] = df['expanded_avg'].apply(format_expansions)
    df['necessary_avg'] = df['necessary_avg'].apply(format_expansions)

    df = df.sort_values(by='alg', key=alg_key_series)
    df_pivot = df.pivot(index='alg', columns='heuristic', values=['expanded_avg', 'necessary_avg', 'instance_count'])

    # Flatten the column multi-index and place 'PDB' heuristic first, followed by metric
    df_pivot.columns = [f"{heuristic}_{metric}" for metric, heuristic in df_pivot.columns]

    # Reset the index for a clean look
    df_pivot = df_pivot.reset_index()
    df_pivot = df_pivot.sort_values(by='alg', key=alg_key_series)

    return df_pivot


def generate_sgr_table(stp_df, dao_df, road_df):
    # Add prefixes to each dataframe's columns
    stp_df = stp_df.add_prefix('stp_')
    dao_df = dao_df.add_prefix('dao_')
    road_df = road_df.add_prefix('road_')

    # Keep the original 'alg' column from each dataframe by renaming them back
    stp_df = stp_df.rename(columns={'stp_alg': 'alg'})
    dao_df = dao_df.rename(columns={'dao_alg': 'alg'})
    road_df = road_df.rename(columns={'road_alg': 'alg'})

    # Merge the dataframes using a full outer join on the 'alg' column
    merged_df = pd.merge(stp_df, dao_df, on='alg', how='outer')
    merged_df = pd.merge(merged_df, road_df, on='alg', how='outer')

    # Replace NaN values in the instance_count columns with -1
    merged_df['stp_instance_count'] = merged_df['stp_instance_count'].fillna(0)
    merged_df['dao_instance_count'] = merged_df['dao_instance_count'].fillna(0)
    merged_df['road_instance_count'] = merged_df['road_instance_count'].fillna(0)

    # Ensure instance_count columns are integers
    merged_df['stp_instance_count'] = merged_df['stp_instance_count'].astype(int)
    merged_df['dao_instance_count'] = merged_df['dao_instance_count'].astype(int)
    merged_df['road_instance_count'] = merged_df['road_instance_count'].astype(int)

    merged_df = merged_df.sort_values(by='alg', key=alg_key_series)

    return merged_df


def generate_toh_dao_table(toh_df, dao_df):
    toh_df = toh_df.add_prefix('toh_')
    dao_df = dao_df.add_prefix('dao_')

    # Keep the original 'alg' column from each dataframe by renaming them back
    dao_df = dao_df.rename(columns={'dao_alg': 'alg'})
    toh_df = toh_df.rename(columns={'toh_alg': 'alg'})

    merged_df = pd.merge(toh_df, dao_df, on='alg', how='outer')

    merged_df = merged_df.sort_values(by='alg', key=alg_key_series)

    return merged_df


def main():
    stp_csvs = [r"results/csvs/journal/stp.csv", r"results/csvs/conference/stp.csv"]
    road_csvs = [r"results/csvs/journal/road.csv", r"results/csvs/conference/road.csv"]
    dao_csvs = [r"results/csvs/journal/dao.csv", r"results/csvs/conference/dao.csv"]
    toh_csvs = [r"results/csvs/journal/toh.csv", r"results/csvs/conference/toh.csv"]
    pancake_csvs = [r"results/csvs/journal/pancake.csv", r"results/csvs/conference/pancake.csv"]

    print("Generating Pancake Latex Table")
    pancake_df = generate_pancake_table(pancake_csvs)
    with open('results/tables/pancake.txt', 'w+') as f:
        pancake_str = multi_heuristic_table(pancake_df, [f'GAP-{x}' for x in range(0, 6)], 50)
        pancake_str = pancake_str.replace('GAP-0', 'GAP')
        f.write(pancake_str)

    print("Generating Pancake Latex Table")
    with open('results/tables/small_pancake.txt', 'w+') as f:
        pancake_str = multi_heuristic_table(pancake_df, [f'GAP-{x}' for x in [0, 1, 3, 5]], 50)
        pancake_str = pancake_str.replace('GAP-0', 'GAP')
        f.write(pancake_str)

    print("Generating ToH Latex Table")
    toh_df = generate_toh_table(toh_csvs)
    with open('results/tables/toh.txt', 'w+') as f:
        toh_str = multi_heuristic_table(toh_df, [f'PDB-{x}' for x in [2, 4, 6]], 50)
        toh_str = toh_str.replace('PDB-2', '(10+2)')
        toh_str = toh_str.replace('PDB-4', '(8+4)')
        toh_str = toh_str.replace('PDB-6', '(6+6)')
        f.write(toh_str)

    print("Generating STP Latex Table")
    stp_df = generate_stp_table(stp_csvs)
    with open('results/tables/stp.txt', 'w+') as f:
        f.write(single_heuristic_table(stp_df, 100))

    print("Generating DAO Latex Table")
    dao_df = generate_dao_table(dao_csvs)
    with open('results/tables/dao.txt', 'w+') as f:
        f.write(single_heuristic_table(dao_df, 3149))

    print("Generating Road Latex Table")
    road_df = generate_road_table(road_csvs)
    with open('results/tables/road.txt', 'w+') as f:
        f.write(single_heuristic_table(road_df, 100))

    print("Generating SGR Latex Table")
    sdr_df = generate_sgr_table(stp_df, dao_df, road_df)
    with open('results/tables/sdr.txt', 'w+') as f:
        f.write(multi_heuristic_table(sdr_df, ['stp', 'dao', 'road'], [100, 3149, 100]))

    print("Generating TOH+DAO Latex Table")
    tohdao_df = generate_toh_dao_table(toh_df, dao_df)
    with open('results/tables/toh_dao.txt', 'w+') as f:
        toh_dao_str = multi_heuristic_table(tohdao_df, ['toh_PDB-2', 'toh_PDB-4', 'toh_PDB-6', 'dao'],
                                            [50, 50, 50, 3149])
        toh_dao_str = toh_dao_str.replace('PDB-2', '(10+2)')
        toh_dao_str = toh_dao_str.replace('PDB-4', '(8+4)')
        toh_dao_str = toh_dao_str.replace('PDB-6', '(6+6)')
        f.write(toh_dao_str)


if __name__ == '__main__':
    main()
