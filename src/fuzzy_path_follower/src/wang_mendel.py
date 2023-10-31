import numpy as np
from T1_set import T1_LeftShoulder, T1_RightShoulder, T1_Triangular
from matplotlib import pyplot as plt
import pandas as pd
from T1_output import T1_LeftShoulder_output, T1_RightShoulder_output, T1_Triangular_output


class wang_mendel(object):

    def __init__(self, antecedent_numbers, linear_range, angular_range, output_range, rules):

        self.distance_antecedents = self.generate_antecedents(
            linear_range, antecedent_numbers[0])

        self.angle_antecedents = self.generate_antecedents(
            angular_range, antecedent_numbers[1])

        self.output_antecedents = self.generate_antecedents(
            output_range, antecedent_numbers[2])

        self.antecedent_numbers = antecedent_numbers

        self.rules = rules

    def plot_antecedents(self):

        fig = plt.figure()
        ax1 = fig.add_subplot(211)
        for i in range(1, self.antecedent_numbers[0] + 1):
            interval = self.distance_antecedents[i].interval

            mf_degrees = self.distance_antecedents[i].get_mf_degrees()

            x = np.linspace(interval[0], interval[1], len(mf_degrees))

            ax1.plot(x, mf_degrees)
            ax1.set_ylabel("Membership")
            ax1.set_xlabel("Distance (m)")
            ax1.grid(True)

        ax2 = fig.add_subplot(212)

        for i in range(1, self.antecedent_numbers[1] + 1):
            interval = self.angle_antecedents[i].interval

            mf_degrees = self.angle_antecedents[i].get_mf_degrees()

            x = np.linspace(interval[0], interval[1], len(mf_degrees))

            ax2.plot(x, mf_degrees)
            ax2.set_ylabel("Membership")
            ax2.set_xlabel("Angle (rad)")
            ax2.grid(True)

        fig.subplots_adjust(hspace=0.5)
        plt.show()

    def plot_output_antecedents(self):
        for i in range(1, self.antecedent_numbers[2] + 1):
            interval = self.output_antecedents[i].interval

            mf_degrees = self.output_antecedents[i].get_mf_degrees()

            x = np.linspace(interval[0], interval[1], len(mf_degrees))

            plt.plot(x, mf_degrees)
            plt.ylabel("Membership")
            plt.grid(True)
        plt.show()

    def generate_antecedents(self, value_range, antecedent_number):
        min_value, max_value = value_range

        antecedents = {}

        step = ((max_value - min_value) /
                (antecedent_number - 1)) / 4.0

        for i in range(1, antecedent_number + 1):

            mean = min_value + (i - 1) * step * 4.0
            if i == 1:
                antecedents[i] = T1_LeftShoulder(
                    mean, step, 500)
            elif i == antecedent_number:
                antecedents[i] = T1_RightShoulder(
                    mean, step, 500)
            else:
                antecedents[i] = T1_Triangular(
                    mean, step, 500)

        return antecedents

    def get_output(self, distance_error, angle_error):
        # Generate_firing_strengths
        matrix_width = max(self.antecedent_numbers)

        distance_antecedent_firings = np.empty(matrix_width)
        angle_antecedent_firings = np.empty(matrix_width)

        for i in range(self.antecedent_numbers[0]):
            distance_fs = self.distance_antecedents[i+1].get_degree(
                distance_error)

            distance_antecedent_firings[i] = distance_fs

        for i in range(self.antecedent_numbers[1]):
            angle_fs = self.angle_antecedents[i+1].get_degree(angle_error)
            angle_antecedent_firings[i] = angle_fs

        firing_strengths = (
            distance_antecedent_firings, angle_antecedent_firings)

        return self.apply_rules_to_input(firing_strengths)

    def apply_rules_to_input(self, firing_strengths):

        rule_output_strength = np.empty([len(self.rules), 1])

        rule_output_strength.fill(np.NaN)

        for rule_index, rule in enumerate(self.rules):
            rule_output_strength[rule_index] = self.individual_rule_output(
                firing_strengths, rule[0:2])

        firing_level_for_each_output = self.union_strength_of_same_antecedents(
            rule_output_strength, self.rules[:, 2])

        return self.generate_outputs_object(firing_level_for_each_output)

    def generate_outputs_object(self, pairs_of_strength_antecedent):

        antecedents = self.output_antecedents

        outputs = {}
        for index_of_ant, fs in pairs_of_strength_antecedent:
            if (isinstance(antecedents[index_of_ant], T1_Triangular)):
                outputs[index_of_ant] = T1_Triangular_output(
                    fs, antecedents[index_of_ant].interval)
            if (isinstance(antecedents[index_of_ant], T1_RightShoulder)):
                outputs[index_of_ant] = T1_RightShoulder_output(
                    fs, antecedents[index_of_ant].interval)
            if (isinstance(antecedents[index_of_ant], T1_LeftShoulder)):
                outputs[index_of_ant] = T1_LeftShoulder_output(
                    fs, antecedents[index_of_ant].interval)

        if (len(outputs) == 0):
            return 0

        degree = []
        try:
            disc_of_all = np.linspace(list(antecedents.values())[0].interval[0],
                                      antecedents[list(
                                          antecedents.keys())[-1]].interval[1],
                                      int((500 / 2.0) * (len(antecedents) + 1)))
        except:
            print("error in generate outputs object")

        for x in disc_of_all:
            max_degree = 0.0
            for i in outputs:
                if max_degree < outputs[i].get_degree(x):
                    max_degree = outputs[i].get_degree(x)
            degree.append(max_degree)

        numerator = np.dot(disc_of_all, degree)
        denominator = sum(degree)
        if not denominator == 0:
            return (numerator / float(denominator))
        else:
            return (0.0)

    def individual_rule_output(self, inputs, rule):
        firing_level_of_pairs = 1
        for i in range(0, len(inputs)):
            temp_firing = inputs[i][int(rule[i]) - 1]

            if (temp_firing == 0):
                firing_level_of_pairs = "nan"
                break

            # minimum is implemented
            if (temp_firing < firing_level_of_pairs):
                firing_level_of_pairs = temp_firing
        return firing_level_of_pairs

    def union_strength_of_same_antecedents(self, list_of_antecedent_strength, output_antecedent_list):
        grouped_output_antecedent_strength = pd.DataFrame(
            index=range(0, len(output_antecedent_list)), columns=range(1, 3))

        grouped_output_antecedent_strength[1] = list_of_antecedent_strength
        grouped_output_antecedent_strength[2] = output_antecedent_list

        l1 = grouped_output_antecedent_strength.groupby([2]).max()

        l1 = pd.DataFrame.dropna(l1)
        return (zip(l1.index, l1[1]))


# np.savetxt(" .csv",self.reduced_rules,delimiter=",")
