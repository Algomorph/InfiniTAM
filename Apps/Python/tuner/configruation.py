import sys
import logging


class GroupedParameter:
    def __init__(self, group, command_line_name):
        self._command_line_name = command_line_name
        self.group = group
        group.parameters.append(self)

    def __get__(self, instance, owner_type):
        return self.value

    def __set__(self, instance, value):
        self.value = value

    @property
    def command_line_string(self):
        value_string = str(self.value)
        if type(self.value) == bool:
            value_string = value_string.lower()
        return self._command_line_name + "=" + value_string


class ParameterGroup:
    def __init__(self):
        self.parameters = []

    def __call__(self, command_line_string):
        return GroupedParameter(self, command_line_string)

    def __get__(self, instance, owner_type):
        return BoundParameterGroup(self, instance, owner_type)


class BoundParameterGroup(object):
    def __init__(self, group, owner_instance, owner_type):
        self.group = group
        self.owner_instance = owner_instance
        self.owner_type = owner_type

    def __dir__(self):
        attribute_names = dir(self.owner_type if self.owner_instance is None else self.owner_instance)
        return [name for name in attribute_names if
                getattr(self.owner_type.__dict__.get(name, None),
                        'group', None) is self.group]

    @property
    def dictionary(self):
        if self.owner_instance is None:
            return {}
        parameter_names = dir(self)
        return {parameter_name: self.owner_type.__dict__.get(parameter_name, None) for parameter_name in
                parameter_names}

    @property
    def parameters(self):
        return self.group.parameters

    def to_command_line_string(self):
        if self.owner_instance is None:
            return ""
        command_line_string = ""
        for parameter in self.group.parameters:
            command_line_string += parameter.command_line_string
        return command_line_string


class LevelSetEvolutionSwitches:
    parameters = ParameterGroup()
    enable_data_term = parameters(" --level_set_evolution.switches.enable_data_term")
    enable_level_set_term = parameters(" --level_set_evolution.switches.enable_level_set_term")
    enable_smoothing_term = parameters(" --level_set_evolution.switches.enable_smoothing_term")
    enable_Killing_field = parameters(" --level_set_evolution.switches.enable_Killing_field")
    enable_Sobolev_gradient_smoothing = \
        parameters(" --level_set_evolution.switches.enable_Sobolev_gradient_smoothing")

    def __init__(self, enable_data_term, enable_level_set_term, enable_smoothing_term, enable_killing_field,
                 enable_Sobolev_gradient_smoothing):
        self.enable_data_term = enable_data_term
        self.enable_level_set_term = enable_level_set_term
        self.enable_smoothing_term = enable_smoothing_term
        self.enable_Killing_field = enable_killing_field
        self.enable_Sobolev_gradient_smoothing = enable_Sobolev_gradient_smoothing


PROGRAM_SUCCESS = 0
PROGRAM_FAILURE = -1


def main() -> int:
    switches = LevelSetEvolutionSwitches(
        enable_data_term=True,
        enable_level_set_term=False,
        enable_smoothing_term=True,
        enable_killing_field=True,
        enable_Sobolev_gradient_smoothing=False
    )
    print(switches.parameters.dictionary)

    return PROGRAM_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
