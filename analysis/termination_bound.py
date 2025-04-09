import argparse
import pathlib
from typing import Union, List

import numpy as np
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


class RunLog:
    def __init__(self, domain: str, instance_line: str, algorithm_line: str, result_line: str, bound_lines: list[str]):
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
        self.terminators: list[str] = None
        self._parse_instance_line(instance_line)
        self._parse_algorithm_line(algorithm_line)
        self._parse_result_line(result_line)
        if bound_lines:
            self._parse_terminators(bound_lines[-2])

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

    def _parse_terminators(self, last_bound_line):
        # Split while skipping NE and [B]
        data = {item.split('=')[0]: float(item.split('=')[1]) for item in last_bound_line.strip().split()[2:]}
        self.terminators = [k for k, v in data.items() if v >= self.solution]


def parse_file(domain: str, file_path: Union[pathlib.Path, str]):
    runlogs: List[RunLog] = []
    instance_line: str
    alg_line: str

    with open(file_path, 'r') as f:
        bound_lines = []
        for line in f:
            line = line.strip()
            if line.startswith('[I]'):
                instance_line = line
            elif line.startswith('[A]'):
                alg_line = line
            elif line.startswith('[B]'):
                bound_lines.append(line)
            elif line.startswith('[R]'):
                runlogs.append(RunLog(domain, instance_line, alg_line, line, bound_lines))
    return runlogs


def load_directory(log_path_dir: Union[str, pathlib.Path], domain: str) -> List[RunLog]:
    runlogs: List[RunLog] = []
    for file_path in pathlib.Path(log_path_dir).rglob('*'):
        if file_path.is_file() and file_path.suffix in ['.txt', '.out', '.log']:
            runlogs.extend(parse_file(domain, file_path))
    return runlogs


def split_instance(df):
    df['map'] = df.apply(lambda row: row['instance'].split('-')[-3].split('/')[-1].split('.')[0], axis=1)
    df['scen'] = df.apply(
        lambda row: (row['instance'].split('-')[-2] + "-" + row['instance'].split('-')[-1]).replace(', ', ';'), axis=1)
    df = df.drop('instance', axis=1)
    return df


# Function to compute the required value
def compute_value(pair):
    left, right = pair
    total = left + right
    return left / total if total != 0 else np.nan


def generate_terminator_df(runlogs, df):
    terminated_runlogs = [r for r in runlogs if 'BAE' in r.alg and '-a' in r.alg and r.terminators]
    by_heuristic = {}
    for r in terminated_runlogs:
        if r.heuristic not in by_heuristic:
            by_heuristic[r.heuristic] = []
        by_heuristic[r.heuristic].append(r)

    for heuristic in by_heuristic:
        curr_terminated_runlogs = by_heuristic[heuristic]
        unique_algs_terminations = {}
        for r in curr_terminated_runlogs:
            if r.alg not in unique_algs_terminations:
                unique_algs_terminations[r.alg] = [0, 0]
            targeted_bound = r.alg.split('-')[1].upper()
            index = 0 if targeted_bound in r.terminators else 1
            unique_algs_terminations[r.alg][index] += 1

        new_df = pd.DataFrame(list(unique_algs_terminations.keys()), columns=['alg'])
        domain = curr_terminated_runlogs[0].domain
        heuristic = curr_terminated_runlogs[0].heuristic
        new_df[f'{domain}-{heuristic}'] = new_df['alg'].map(lambda x: compute_value(unique_algs_terminations[x]))
        df = df.merge(new_df, on="alg", how="outer")

    return df


def generate_excel(df):
    df.to_excel("results/termination.xlsx", index=False)


def rename_algs(df):
    df['alg'] = df['alg'].apply(lambda x: f"tb-{BOUND_NUMBERS[x.split('-')[1]]}")
    return df


def main():
    df = pd.DataFrame(columns=['alg'])
    # Iterate over the files and parse every file in the given directory
    print("Loaded ToH")
    toh_logs: List[RunLog] = load_directory(r'data/conference/toh', 'toh')
    print("Loaded STP")
    stp_logs: List[RunLog] = load_directory(r'data/conference/stp', 'stp')
    print("Loaded Pancake")
    pancake_logs: List[RunLog] = load_directory(r'data/conference/pancake', 'pancake')
    df = generate_terminator_df(toh_logs, df)
    df = generate_terminator_df(stp_logs, df)
    df = generate_terminator_df(pancake_logs, df)
    df = rename_algs(df)
    df.sort_values(by='alg', key=lambda x: x.str.extract(r'tb-(\d+)').astype(int).iloc[:, 0], inplace=True)
    generate_excel(df)


if __name__ == '__main__':
    main()
