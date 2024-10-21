from pathlib import Path
from typing import List, Tuple

import matplotlib
import matplotlib.pyplot as plt


class InstanceLog:
    def __init__(self, instance_line, result_line, alg_line, bound_lines):
        self.heuristic = None
        self.alg = None
        self.instance = -1
        self.solution = -1
        self.expanded = -1
        self.necessary = -1
        self.time = -1
        self.bounds = []
        self._parse_file(instance_line, result_line, alg_line, bound_lines)

    def _parse_file(self, instance_line, result_line, alg_line, bound_lines):
        instance_line = instance_line.split()
        self.instance = int(instance_line[4])
        self.heuristic = instance_line[1]

        result_line = result_line.split()
        self.solution = float(result_line[3][:-1])
        self.expanded = int(result_line[4])
        self.necessary = int(result_line[6])
        self.time = float(result_line[8][:-1])

        self.alg = alg_line.split()[1][:-1]

        for line in bound_lines:
            line = line.split()
            if line[1] == 'Done':
                break
            self.bounds.append({key: float(value) for key, value in [_.split('=') for _ in line[1:]]})


def load_dir(dir_path, algs):
    ils = []
    instance_line = None
    result_line = None
    alg_line = None
    bound_lines = []
    include_flag = False
    for fpath in Path(dir_path).iterdir():
        if fpath.is_file():
            with open(fpath, 'r') as f:
                for line in f:
                    if line.startswith('[I]'):
                        instance_line = line
                    elif line.startswith('[A]'):
                        alg_line = line
                        include_flag = alg_line.split()[1][:-1] in algs
                    elif line.startswith('[R]'):
                        result_line = line
                        if include_flag:
                            ils.append(InstanceLog(instance_line, result_line, alg_line, bound_lines))
                        bound_lines.clear()
                    elif line.startswith('[B]'):
                        bound_lines.append(line)
    return ils


def get_indexes_to_advance(total_xs: List[List[float]], indexes: List[int]):
    # Find the minimal x value of the next step for each instance
    curr_min = min([total_xs[i][indexes[i] + 1] for i in range(len(indexes))])
    # We need to increment only those, as they are doing the next step in their bounds progress
    return [i for i in range(len(indexes)) if total_xs[i][indexes[i] + 1] == curr_min]


def generate_plot_x_y(instance_logs: List[InstanceLog], bound: str):
    indexes: List[int] = [0] * len(instance_logs)
    total_xs: List[List[float]] = []
    total_ys: List[List[float]] = []
    final_list: List[Tuple[float, float]] = []
    for i, instance_log in enumerate(instance_logs):
        # Normalize the x-values to be % of the progress towards the final expansion
        curr_xs: List[float] = [bounds_map['NE'] / instance_log.expanded for bounds_map in instance_log.bounds]
        # Normalize the y-values to be a % of C* for that instance (note that it can get above 1)
        curr_ys: List[float] = [bounds_map[bound] / instance_log.solution for bounds_map in instance_log.bounds]
        # In the case where the bound stops before the last expansion, we add this to the draw the step until the end
        if curr_xs[-1] != 1:
            curr_xs += [1]
            curr_ys += [curr_ys[-1]]
        total_xs.append(curr_xs)
        total_ys.append(curr_ys)

    # To create a step of the combined list, we take the max of the current x's, and the mean of the y's.
    # We want to catch the average bound value as progress of expansions. Therefore, we are only interested in areas
    # where something changes, therefore we always look for the minimal change (to catch them all), i.e., the maximum of
    # the next x's for each instance
    final_list.append((max([total_xs[i][indexes[i]] for i in range(len(indexes))]),
                       sum([total_ys[i][indexes[i]] for i in range(len(indexes))]) / len(indexes)))

    while any([indexes[i] + 1 != len(total_xs[i]) for i in range(len(indexes))]):
        indexes_to_advance = get_indexes_to_advance(total_xs, indexes)
        for i in indexes_to_advance:
            indexes[i] += 1
        final_list.append((max([total_xs[i][indexes[i]] for i in range(len(indexes))]),
                           sum([total_ys[i][indexes[i]] for i in range(len(indexes))]) / len(indexes)))
    return final_list


def generate_full_plot(xy_lists: List[List[List[Tuple[int, int]]]], output_file: str = None):
    # Linestyles and colors to differentiate the graphs and keep them consistent
    linestyles = ['-', '--', ':', '-.']  # , (0, (5, 10)), (0, (1, 1)), (0, (3, 5, 1, 5))]
    colors = ['#000000', '#E69F00', '#56B4E9', '#009E73', '#D55E00', '#0072B2', '#CC79A7']

    # Draw the figure
    fig, axs = plt.subplots(2, 2, figsize=(10, 8), sharex=True, sharey=True)

    # Set common labels
    fig.text(0.5, 0.04, 'Search progress', ha='center', va='center', fontsize=20, fontweight='bold')
    fig.text(0.04, 0.5, 'Bound / C*', ha='center', va='center', rotation='vertical', fontsize=20, fontweight='bold')

    # adjust distance between subplots since the ticks overlap
    plt.subplots_adjust(wspace=0.09, hspace=0.06)

    for i, sublist in enumerate(xy_lists):
        for j, subsublist in enumerate(sublist):
            # Separate x and y values
            x_values = [point[0] for point in subsublist]
            y_values = [point[1] for point in subsublist]

            # Plot each sublist
            curr_ax = axs[(i // 2), (i % 2)]
            curr_ax.plot(x_values, y_values, linestyle=linestyles[j % len(linestyles)], linewidth=4,
                         color=colors[j % len(colors)], label=f'$\mathbf{{B_{j + 1}}}$')
            curr_ax.set_xlim(0, 1)
            curr_ax.set_ylim(0.2, 1)
            # Mark each subplot with a,b,c,d to reference in the paper
            curr_ax.text(0.02, 0.98, chr(ord('a') + i), transform=curr_ax.transAxes, fontsize=16, fontweight='bold',
                         va='top')

            # Set tick parameters to bold
            curr_ax.tick_params(axis='both', which='major', labelsize=12, width=2)

            # Set bold for existing tick labels
            for label in curr_ax.get_xticklabels(): label.set_fontweight('bold')

            for label in curr_ax.get_yticklabels(): label.set_fontweight('bold')

            curr_ax.grid(True)

    # Set algorithm titles
    axs[0, 0].set_title('$\mathbf{TB_4}$', fontweight='bold', fontsize=20)
    axs[0, 1].set_title('$\mathbf{TB_7}$', fontweight='bold', fontsize=20)

    # Set the heuristics, reverse the text and then distance it from the subplot border
    axs[0, 1].yaxis.set_label_position('right')
    axs[0, 1].set_ylabel('GAP-1', rotation=270, labelpad=22, fontweight='bold', fontsize=20)

    axs[1, 1].yaxis.set_label_position('right')
    axs[1, 1].set_ylabel('GAP-5', rotation=270, labelpad=22, fontweight='bold', fontsize=20)

    handles, labels = axs[1, 1].get_legend_handles_labels()

    # Reorder the handles and labels to be in row-major order
    order = [0, 4, 1, 5, 2, 6, 3]  # Define a custom order for row-major
    handles = [handles[idx] for idx in order]
    labels = [labels[idx] for idx in order]

    # Use the reordered handles and labels
    axs[1, 1].legend(handles, labels, ncol=4, prop={'weight': 'bold'}, loc='lower center')

    if output_file:
        plt.savefig(output_file, dpi=200, bbox_inches='tight', pad_inches=0)
    else:
        plt.show()


def plot(xy_lists, labels):
    linestyles = ['-', '--', ':', '-.', (0, (5, 10)), (0, (1, 1)), (0, (3, 5, 1, 5))]
    colors = ['#000000', '#E69F00', '#56B4E9', '#009E73', '#D55E00', '#0072B2', '#CC79A7']

    plt.figure()
    plt.xlim(0, 1)
    plt.ylim(0.25, 1)

    # Iterate through each sublist
    for i, sublist in enumerate(xy_lists):
        # Separate x and y values
        x_values = [point[0] for point in sublist]
        y_values = [point[1] for point in sublist]

        # Plot each sublist
        plt.plot(x_values, y_values,
                 linestyle=linestyles[i % len(linestyles)], linewidth=4, color=colors[i % len(colors)],
                 label=labels[i])

    # Add labels and show the plot
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Plot of lists of x-y pairs')
    plt.legend()
    plt.grid(True)
    plt.show()


def main():
    logs_dir = r"data/conference/pancake"
    bounds = ['FD', 'DF', 'G', 'B', 'GFGD', 'GDGF', 'GB']
    plot_data = []
    ils = load_dir(logs_dir, ['BAE*-b-a', 'BAE*-gb-a'])
    ils = [il for il in ils if il.heuristic in ['GAP-1', 'GAP-5']]
    for alg in ['BAE*-b-a', 'BAE*-gb-a']:
        for heuristic in ['GAP-1', 'GAP-5']:
            plot_data.append([None] * 7)
            for i, bound in enumerate(bounds):
                plot_data[-1][i] = generate_plot_x_y([il for il in ils if il.heuristic == heuristic and il.alg == alg],
                                                     bound)
    generate_full_plot(plot_data, 'results/bounds_progression.pdf')


if __name__ == '__main__':
    main()
