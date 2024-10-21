import pathlib

import numpy as np
import pandas as pd

from agg_journal_logs import parse_dir


def print_df(df: pd.DataFrame, max_rows=None, max_cols=None):
    with pd.option_context('display.max_rows', max_rows, 'display.max_columns', max_cols):
        print(df)


def get_csbs_weights(e1, e2, e3, method):
    if method == 'difference':
        w1 = (e1 + e2 + e3) - e1
        w2 = (e1 + e2 + e3) - e2
        w3 = (e1 + e2 + e3) - e3

        # Normalize the results such that their sum is 1
        relative_w1 = w1 / (w1 + w2 + w3)
        relative_w2 = w2 / (w1 + w2 + w3)
        relative_w3 = w3 / (w1 + w2 + w3)
        return relative_w1, relative_w2, relative_w3
    elif method == 'inverse':
        w1 = 1 / e1
        w2 = 1 / e2
        w3 = 1 / e3

        # Normalize the results such that their sum is 1
        relative_w1 = w1 / (w1 + w2 + w3)
        relative_w2 = w2 / (w1 + w2 + w3)
        relative_w3 = w3 / (w1 + w2 + w3)
        return relative_w1, relative_w2, relative_w3


def verify_column_values(df, group_cols, check_col):
    """
    This method is helper method that verifies that when grouping by group_cols, there is only 1 unique value in
    check_col for every group_cols combination. This can be used for example to verify that all instances have the same
    solution length regardless of algorithm parameters
    """
    # Group by the specified columns
    grouped = df.groupby(group_cols)[check_col]
    # Check if each group has only one unique value
    consistent = grouped.nunique() == 1
    # All groups should have consistent values
    return consistent.all()


def verify_matching_solution_length(df):
    if not verify_column_values(df, ['instance'], 'solution'):
        raise ValueError('Some instances appear with two different solutions')


def get_weight(df):
    # Helper function to remove the scientific notation
    def format_float(num):
        return np.format_float_positional(num, trim='-')

    # Filtering so that we are only left with "training" data. Note that the test is 0<=i<=49 because this is what we
    # ran all algorithms on
    df = df[df['instance'] >= 50]
    algs = ['CSBS-1-0-0', 'CSBS-0-1-0', 'CSBS-0-0-1']
    # Drop unnecessary columns for the current calculation
    df = df.drop(['solution', 'necessary', 'time', 'instance'], axis=1)
    df = df.groupby(['heuristic', 'alg'], as_index=False).sum()
    gaps = df['heuristic'].unique().tolist()
    for gap in gaps:
        # Since we want a small number of expansions to have larger power, instead of x, we take 1/x
        print(f"-------------------- GAP {gap} ------------------------------")
        e1 = df[(df['heuristic'] == gap) & (df['alg'] == algs[0])].iloc[0]['expansions']
        e2 = df[(df['heuristic'] == gap) & (df['alg'] == algs[1])].iloc[0]['expansions']
        e3 = df[(df['heuristic'] == gap) & (df['alg'] == algs[2])].iloc[0]['expansions']
        print(e1, e2, e3)
        relative_w1, relative_w2, relative_w3 = get_csbs_weights(e1, e2, e3, 'inverse')
        print("inverse weights:", format_float(relative_w1), format_float(relative_w2), format_float(relative_w3))

        relative_w1, relative_w2, relative_w3 = get_csbs_weights(e1, e2, e3, 'difference')
        print("difference weights:", format_float(relative_w1), format_float(relative_w2), format_float(relative_w3))


def main():
    output_dir = pathlib.Path(r'C:\Users\LiorS\My Drive\Projects\F2E Journal\Results\pancake_results.xlsx')
    domain_dir = pathlib.Path(r'C:\Users\LiorS\My Drive\Projects\F2E Journal\Data\pancake')
    log_list = parse_dir(domain_dir)
    df = pd.DataFrame([vars(log) for log in log_list])
    verify_matching_solution_length(df)
    get_weight(df)
    # df = df[df['instance'] < 50]
    # df.to_excel(output_dir, index=False)


if __name__ == '__main__':
    main()
