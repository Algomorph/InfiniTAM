import sys
import logging


class GroupedParameter:
    def __init__(self, group, value, command_line_name):
        self.group = group
        self.value = value
        self.name = None
        self.command_line_string = command_line_name

    def __get__(self, instance, owner_type=None):
        return self.value

    def __set__(self, instance, value):
        self.value = value

    def to_command_line_string(self):
        value_string = str(self.value)
        if type(self.value) == bool:
            value_string = value_string.lower()
        return self.command_line_string + "=" + value_string


class ParameterGroup:
    def __init__(self):
        self.__parameters = []
        self.instance = None

    def __call__(self, parameter_value, command_line_name):
        parameter_value = GroupedParameter(self, parameter_value, command_line_name)
        self.__parameters.append(parameter_value)
        return parameter_value

    def to_command_line_string(self):
        result = self.__parameters[0].to_command_line_string()
        for i_parameter in range(1, len(self.__parameters)):
            result += " " + self.__parameters[i_parameter].to_command_line_string()
        return result


class LevelSetEvolutionSwitches:

    def __init__(self, enable_data_term, enable_level_set_term, enable_smoothing_term, enable_killing_field,
                 enable_sobolev_gradient_smoothing):
        parameter_group = ParameterGroup()
        self.parameter_group = parameter_group
        self.enable_data_term = parameter_group(enable_data_term,
                                                " --level_set_evolution.switches.enable_data_term")
        self.enable_level_set_term = parameter_group(enable_level_set_term,
                                                     " --level_set_evolution.switches.enable_level_set_term")
        self.enable_smoothing_term = parameter_group(enable_smoothing_term,
                                                     " --level_set_evolution.switches.enable_smoothing_term")
        self.enable_Killing_field = parameter_group(enable_killing_field,
                                                    " --level_set_evolution.switches.enable_Killing_field")
        self.enable_sobolev_gradient_smoothing = \
            parameter_group(enable_sobolev_gradient_smoothing,
                            " --level_set_evolution.switches.enable_sobolev_gradient_smoothing")

    def to_command_line_string(self):
        return self.parameter_group.to_command_line_string()

    def to_dictionary(self):
        return self.parameter_group.to_dictionary()

    def __repr__(self):
        return self.parameter_group.to_command_line_string()


PROGRAM_SUCCESS = 0
PROGRAM_FAILURE = -1


def main() -> int:
    switches = LevelSetEvolutionSwitches(
        enable_data_term=True,
        enable_level_set_term=False,
        enable_smoothing_term=True,
        enable_killing_field=True,
        enable_sobolev_gradient_smoothing=False
    )
    print(switches.to_command_line_string())
    print(switches.to_dictionary())
    return PROGRAM_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
