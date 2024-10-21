import pathlib
import argparse
import warnings
from typing import Union, List

import pandas as pd

KORF_SOLUTIONS = [57, 55, 59, 56, 56, 52, 52, 50, 46, 59,
                  57, 45, 46, 59, 62, 42, 66, 55, 46, 52,
                  54, 59, 49, 54, 52, 58, 53, 52, 54, 47,
                  50, 59, 60, 52, 55, 52, 58, 53, 49, 54,
                  54, 42, 64, 50, 51, 49, 47, 49, 59, 53,
                  56, 56, 64, 56, 41, 55, 50, 51, 57, 66,
                  45, 57, 56, 51, 47, 61, 50, 51, 53, 52,
                  44, 56, 49, 56, 48, 57, 54, 53, 42, 57,
                  53, 62, 49, 55, 44, 45, 52, 65, 54, 50,
                  57, 57, 46, 53, 50, 49, 44, 54, 57, 54]


class RunLog:
    def __init__(self, domain: str, heuristic: str, instance_id: int, result_line: str):
        self.domain: str = domain
        self.heuristic: str = heuristic
        self.instance: int = instance_id
        self.alg: str
        self.solution: float = 1
        self.necessary: int = -1
        self.expansions: int = -1
        self.time: float = -1
        self.alg: str = ''
        self._parse_result_line(result_line)

    def _parse_result_line(self, line: str) -> None:
        line = line.split()
        self.alg = line[1][:-1]
        self.solution = float(line[3][:-1])
        if self.domain == 'STP4' and KORF_SOLUTIONS[self.instance] != self.solution:
            raise Exception(f'Instance {self.instance} with {self.alg} has non-optimal solution')
        self.expansions = int(line[5][:-1])
        self.necessary = int(line[7][:-1])
        self.time = float(line[9][:-1])


def parse_file(file_path: Union[pathlib.Path, str]) -> List[RunLog]:
    runlogs: List[RunLog] = []
    curr_instance_id: int = -1
    heuristic: str = ''
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('[D]'):
                domain = line.split()[1]
                heuristic = line.split()[2]
            else:
                if line.startswith('[I]'):
                    curr_instance_id = int(line.split()[2].replace(':', ''))
                elif line.startswith('[R]'):
                    runlogs.append(RunLog(domain, heuristic, curr_instance_id, line))
    return runlogs


def parse_dir(dir_path: Union[pathlib.Path, str]) -> List[RunLog]:
    dir_path: pathlib.Path = pathlib.Path(dir_path)
    runlogs: List[RunLog] = []
    for file_path in dir_path.rglob('*'):
        if file_path.is_file() and file_path.suffix in ['.log', '.txt', '.out']:
            runlogs.extend(parse_file(file_path))
    return runlogs


def verify_instance_same_count(df: pd.DataFrame, throw_exception=False, verbose=False, silent=True):
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
            if not silent:
                warnings.warn(error_output)


def verify_single_solution_per_instance(df: pd.DataFrame, allowed: int = 1,
                                        groupby: Union[List[str], str] = 'instance'):
    df = df[(df['alg'] != 'GMX') & (df['alg'] != 'NGMX')]
    unique_solutions_per_instance = df.groupby(groupby)['solution'].nunique()
    if (unique_solutions_per_instance > allowed).any():
        print(unique_solutions_per_instance)
        raise Exception('There exists an instance where two optimal algorithms gave different solutions')


def main():
    # Make an argparser
    parser = argparse.ArgumentParser(description="Process some parameters.")
    parser.add_argument('-l', '--log_dir', type=str, required=True, help='Directory where logs are stored')
    parser.add_argument('-d', '--domain', type=str, choices=['road', 'dao', 'pancake', 'toh', 'stp'],
                        required=True, help='Domain name')
    parser.add_argument('-o', '--output', type=str, default=None, help='Output file path')
    args = parser.parse_args()

    # Get the arguments in variables for easy usage
    domain: str = args.domain
    log_path_dir: pathlib.Path = pathlib.Path(args.log_dir)
    if args.output:
        output_path: str = args.output
    else:
        output_path: str = f'{domain}_result.csv'

    runlogs = parse_dir(args.log_dir)


    data_list = [
        {key: getattr(obj, key) for key in dir(obj) if not callable(getattr(obj, key)) and not key.startswith("__")} for
        obj in runlogs]

    df = pd.DataFrame(data_list)
    verify_single_solution_per_instance(df, groupby=['heuristic','instance'])
    verify_instance_same_count(df)

    df.to_csv(output_path, index=False)



if __name__ == '__main__':
    main()
