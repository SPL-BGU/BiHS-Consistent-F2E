import argparse
import pathlib
import warnings
from typing import Union, List

import pandas as pd


class RunLog:
    def __init__(self, domain: str, instance_line: str, algorithm_line: str, result_line: str):
        self.domain = domain
        self.instance: Union[int, str] = -1
        self.heuristic: str
        if domain == 'road':
            self.heuristic = 'SLD'
        elif domain == 'dao':
            self.heuristic = 'MD'
        else:
            self.heuristic = 'NaN'
        self.solution: float = -1
        self.expanded: int = -1
        self.necessary: int = -1
        self.time: float = -1
        self.alg: str = "NaN"
        self._parse_instance_line(instance_line)
        self._parse_algorithm_line(algorithm_line)
        self._parse_result_line(result_line)

    def _parse_instance_line(self, instance_line: str):
        if self.domain in ['toh', 'stp', 'pancake']:
            instance_line = instance_line.split()
            self.heuristic = instance_line[1]
            if self.domain == 'stp':
                self.instance = int(instance_line[3])
            else:
                self.instance = int(instance_line[4])
        else:
            self.instance = instance_line.split(None, 1)[1]

    def _parse_algorithm_line(self, alg_line: str):
        self.alg = alg_line.split()[1][:-1]

    def _parse_result_line(self, result_line: str):
        line = result_line.split()
        if line[2] == 'found':
            self.solution = float(line[5][:-1])
            self.expanded = int(line[6])
            self.necessary = int(line[8])
            self.time = float(line[10][:-1])
        else:
            self.solution = float(line[3][:-1])
            self.expanded = int(line[4])
            self.necessary = int(line[6])
            self.time = float(line[8][:-1])


def parse_file(domain: str, file_path: Union[pathlib.Path, str]):
    runlogs: List[RunLog] = []
    instance_line: str
    alg_line: str

    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('[I]'):
                instance_line = line
            elif line.startswith('[A]'):
                alg_line = line
            elif line.startswith('[R]'):
                runlogs.append(RunLog(domain, instance_line, alg_line, line))
    return runlogs


def load_directory(log_path_dir: Union[str, pathlib.Path], domain: str) -> List[RunLog]:
    runlogs: List[RunLog] = []
    for file_path in log_path_dir.rglob('*'):
        if file_path.is_file() and file_path.suffix in ['.txt', '.out', '.log']:
            runlogs.extend(parse_file(domain, file_path))
    return runlogs


def verify_single_solution_per_instance(df: pd.DataFrame, allowed=1):
    df = df[(df['alg'] != 'GMX') & (df['alg'] != 'NGMX')]
    unique_solutions_per_instance = df.groupby('instance')['solution'].nunique()
    if (unique_solutions_per_instance > allowed).any():
        print(unique_solutions_per_instance)
        raise Exception('There exists an instance where two optimal algorithms gave different solutions')


def verify_instance_same_count(df: pd.DataFrame, throw_exception=False, verbose=False):
    grouped_counts = df.groupby(['alg', 'heuristic']).size().reset_index(name='counts')
    majority_count = grouped_counts['counts'].mode()[0]
    non_majority_groups = grouped_counts[grouped_counts['counts'] != majority_count]
    if not non_majority_groups.empty:
        if verbose:
            non_majority_groups.sort_values(by=['heuristic', 'counts'], inplace=True)
            error_output = ("Following groups have none-equal number of instances:\n" +
                            non_majority_groups.to_string(index=False))
        else:
            error_output = "There are algorithms with different instance count than the rest"
        if throw_exception:
            raise Exception(error_output)
        else:
            warnings.warn(error_output)


def split_instance(df):
    df['map'] = df.apply(lambda row: row['instance'].split('-')[-3].split('/')[-1].split('.')[0], axis=1)
    df['scen'] = df.apply(
        lambda row: (row['instance'].split('-')[-2] + "-" + row['instance'].split('-')[-1]).replace(', ', ';'), axis=1)
    df = df.drop('instance', axis=1)
    return df


def main():
    # Make an argparser
    parser = argparse.ArgumentParser(description="Process some parameters.")
    parser.add_argument('-l', '--log_dir', type=str, required=True, help='Directory where logs are stored')
    parser.add_argument('-d', '--domain', type=str, choices=['road', 'dao', 'pancake', 'toh', 'stp'],
                        required=True, help='Domain name')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='Output file path')
    args = parser.parse_args()

    # Get the arguments in variables for easy usage
    domain: str = args.domain
    log_path_dir: pathlib.Path = pathlib.Path(args.log_dir)
    if args.output:
        output_path: str = args.output
    else:
        output_path: str = f'{domain}_result.csv'

    # Iterate over the files and parse every file in the given directory
    runlogs: List[RunLog] = load_directory(log_path_dir, domain)
    data_list = [
        {key: getattr(obj, key) for key in dir(obj) if not callable(getattr(obj, key)) and not key.startswith("__")} for
        obj in runlogs]

    df = pd.DataFrame(data_list)

    verify_single_solution_per_instance(df, 2 if domain == 'dao' else 1)

    # Fix the MaxTot issue with the conference results
    df_filtered = df[~df['alg'].str.contains('MaxTot', case=False, na=False)]

    df_filtered.to_csv(output_path, index=False)


if __name__ == '__main__':
    main()
