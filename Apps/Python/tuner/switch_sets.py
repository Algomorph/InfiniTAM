from configruation import LevelSetEvolutionSwitches

level_set_evolution_switches_killing_level_set = LevelSetEvolutionSwitches(
    enable_data_term=True,
    enable_level_set_term=True,
    enable_smoothing_term=True,
    enable_killing_field=True,
    enable_sobolev_gradient_smoothing=False
)

level_set_evolution_switches_killing = LevelSetEvolutionSwitches(
    enable_data_term=True,
    enable_level_set_term=False,
    enable_smoothing_term=True,
    enable_killing_field=True,
    enable_sobolev_gradient_smoothing=False
)

level_set_evolution_switches_sobolev = LevelSetEvolutionSwitches(
    enable_data_term=True,
    enable_level_set_term=False,
    enable_smoothing_term=True,
    enable_killing_field=False,
    enable_sobolev_gradient_smoothing=True
)

level_set_evolution_switches_tikhonov = LevelSetEvolutionSwitches(
    enable_data_term=True,
    enable_level_set_term=False,
    enable_smoothing_term=True,
    enable_killing_field=False,
    enable_sobolev_gradient_smoothing=False
)