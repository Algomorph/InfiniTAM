import pyglet.canvas
import vtk
import ruamel.yaml
from typing import List


def find_screen(screens: List[pyglet.canvas.xlib.XlibScreen], x: int, y: int) -> int:
    """
    Find the screen where the given display coordinate resides from a sorted list of screens.
    Assumes
    :param screens: a list of screens assumed to be sorted by their coordinate, i.e. (screen.x, screen.y)
    :param x: display x-coordinate
    :param y: display y-coordinate
    :return: index of the screen from the list, -1 if the coordinate is not on any of the screens in the list
    """
    screen_index = 0
    for screen in screens:
        if x < screen.x + screen.width and y < screen.y + screen.height:
            break
        screen_index += 1
    if x >= screens[-1].x + screens[-1].width or y >= screens[-1].y + screens[-1].height:
        return -1
    return screen_index


def set_up_render_window_bounds(render_window: vtk.vtkRenderWindow, configuration: dict = None, default_screen_index: int = 3) -> None:
    display = pyglet.canvas.get_display()
    screens = display.get_screens()
    screens.sort(key=lambda screen: (screen.x, screen.y))

    if configuration is not None and "screen_index" in configuration:
        chosen_screen_index = configuration["screen_index"]
    else:
        chosen_screen_index = min(len(screens) - 1, default_screen_index)
    chosen_screen = screens[chosen_screen_index]

    if configuration is not None:
        if "screen_x" in configuration and "screen_y" in configuration:
            window_x, window_y = configuration["window_x"], configuration["window_y"]
            render_window.SetPosition(window_x, window_y)
            screen_index_based_on_coordinate = find_screen(screens, window_x, window_y)
            chosen_screen = screens[screen_index_based_on_coordinate]
            if "screen_index" in configuration and chosen_screen_index != screen_index_based_on_coordinate:
                raise Warning("screen_index setting in given configuration is {0:d}, which contradicts window_x and window_y "
                              "parameters ({1:d} and {2:d}, respectively) in the configuration, which correspond to screen {3:d}."
                              "Using screen {3:d}, derived from window_x and window_y parameters."
                              .format(chosen_screen_index, window_x, window_y, screen_index_based_on_coordinate))

            if "window_fullscreen" in configuration and configuration["window_fullscreen"] is True:
                render_window.SetSize((chosen_screen.width, chosen_screen.height))
            elif "window_width" in configuration and "window_height" in configuration:
                render_window.SetSize((configuration["window_width"], configuration["window_height"]))
            else:
                # attempt to either center the window in the screen or draw to the far corner of the screen
                screen_x = configuration["window_x"] - chosen_screen.x
                screen_y = configuration["window_y"] - chosen_screen.y
                window_width = chosen_screen.width - screen_x * 2
                if window_width < 400:
                    # too small, go to the corner
                    window_width = chosen_screen.width - screen_x
                window_height = chosen_screen.height - screen_y * 2
                if window_height < 300:
                    # too small, go to the corner
                    window_height = chosen_screen.height - screen_y
                render_window.SetSize((window_width, window_height))
        else:
            if "window_fullscreen" in configuration and configuration["window_fullscreen"] is True:
                render_window.SetPosition(chosen_screen.x, chosen_screen.y)
                render_window.SetSize((chosen_screen.width, chosen_screen.height))
            elif "window_width" in configuration and "window_height" in configuration:
                # try to center the window on the screen
                render_window.SetPosition(chosen_screen.x + (chosen_screen.width - configuration["window_width"]) // 2,
                                          chosen_screen.y + (chosen_screen.height - configuration["window_height"]) // 2)
                render_window.SetSize((configuration["window_width"], configuration["window_height"]))
            else:
                render_window.SetPosition(chosen_screen.x, chosen_screen.y)
                render_window.SetSize((chosen_screen.width, chosen_screen.height))
    else:
        render_window.SetPosition(chosen_screen.x, chosen_screen.y)
        render_window.SetSize((chosen_screen.width, chosen_screen.height))
