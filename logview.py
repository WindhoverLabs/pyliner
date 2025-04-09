"""
Pyliner LogView

Enables the fast and easy browsing of Pyliner logs.

Usage:
    python logview.py [filename]

"""

import argparse
import curses
import glob
import os
import re
from abc import abstractmethod

from future.moves import itertools

from pyliner.util import query_yes_no

print_errors = []


class LogLine(object):
    """A line in a log file."""
    def __init__(self, log_line, text):
        self.log_line = log_line
        self.text = text

    @property
    def level(self):
        """Return the curses color-pair number for the level of the line."""
        try:
            level = self.text.split(' - ')[2]
        except IndexError:
            level = 'CRITICAL'
        table = {
            'NOTSET': curses.COLOR_WHITE,
            'DEBUG': curses.COLOR_WHITE,
            'INFO': curses.COLOR_WHITE,
            'WARNING': curses.COLOR_YELLOW,
            'ERROR': curses.COLOR_RED,
            'CRITICAL': curses.COLOR_RED
        }
        return table[level]


class FilterLine(LogLine):
    """A line that has been filtered."""
    def __init__(self, filter_line, log_line):
        super(FilterLine, self).__init__(log_line.log_line, log_line.text)
        self.filter_line = filter_line


class ViewLine(FilterLine):
    """A line that is in view."""
    def __init__(self, view_line, filter_line):
        super(ViewLine, self).__init__(filter_line.filter_line, filter_line)
        self.view_line = view_line


class LogView(object):
    FG_SHIFT = 8

    def __init__(self, stdscr, log_name):
        # Curses Initialization
        # Start colors in curses
        curses.start_color()
        colors = [curses.COLOR_BLACK, curses.COLOR_RED, curses.COLOR_GREEN,
                  curses.COLOR_YELLOW, curses.COLOR_BLUE, curses.COLOR_MAGENTA,
                  curses.COLOR_CYAN, curses.COLOR_WHITE]
        for fg, bg in itertools.permutations(colors, 2):
            curses.init_pair(fg*LogView.FG_SHIFT + bg, fg, bg)
        self.colors = colors

        # Application State
        self.exit = False
        self.log_lines = None
        self.log_name = log_name
        self.state = None
        self.state_map = None
        self.stdscr = stdscr

        with open(self.log_name) as fp:
            self.log_lines = [LogLine(*x) for x in enumerate(fp.readlines())]

        self.state_map = {
            'NORMAL': NormalState,
            'GOTO': GotoState
        }

        # Useful for User Interaction
        self.filter = ''
        self.filter_lines = None
        """:type: list[FilterLine]"""
        self.apply_filter(lambda line: True)
        self.view_max = 1
        self.view_min = 0
        self.cursor_column = 0
        self.cursor_line = 0

    def apply_filter(self, func):
        """Set filter_lines to the lines filtered by func, and global_cursor
        to the closest previous filtered line."""
        if self.filter is func:
            return
        self.filter = func
        if isinstance(func, str):
            func = lambda line: line.text.contains(func)
        self.filter_lines = [FilterLine(*x) for x in 
                             enumerate(filter(func, self.log_lines))]
    
    def center_view(self, filter_index):
        """Set view to contain lines centered around filter_index as much
        as possible."""
        h = self.height
        h2 = h / 2
        num_filter = len(self.filter_lines)
        min_index = max(0, filter_index - h2)
        max_index = min(num_filter, min_index + h)
        # if max_index == num_filter:
        #     min_index = num_filter - h
        # if min_index < 0:
        #     max_index = h - 1
        # self.view_lines = [ViewLine(*x) for x in
        #                    enumerate(self.filter_lines[min_index: max_index])]\
        self.view_max = max_index
        self.view_min = min_index

    def cursor_move(self, lines):
        self.cursor_line += lines
        self.cursor_line = max(0, self.cursor_line)
        self.cursor_line = min(len(self.filter_lines)-1, self.cursor_line)
        self.put_line_in_view(self.cursor_line)
        
    @staticmethod
    def get_color(fg, bg):
        return curses.color_pair(fg*LogView.FG_SHIFT + bg)

    @property
    def height(self):
        return self.stdscr.getmaxyx()[0]

    def put_line_in_view(self, filter_index):
        """Puts filter_index in view."""
        cur_min = self.view_min
        cur_max = self.view_max
        if cur_min < filter_index < cur_max:
            return
        h = self.height
        if filter_index < cur_min:
            self.view_min = filter_index
            self.view_max = self.view_min + h - 1
        elif filter_index > cur_max:
            self.view_max = filter_index + 1
            self.view_min = self.view_max - h

    def key_press(self, key):
        if self.state.handle_key(key):
            return

        # Key Bindings
        x_vel = 1
        y_vel = 2

        handled = True
        if key == curses.KEY_DOWN:
            self.cursor_move(x_vel)
        elif key == curses.KEY_UP:
            self.cursor_move(-x_vel)
        elif key == curses.KEY_NPAGE:
            self.cursor_move(10 * x_vel)
        elif key == curses.KEY_PPAGE:
            self.cursor_move(-10 * x_vel)
        elif key == curses.KEY_LEFT:
            self.cursor_column -= y_vel
        elif key == curses.KEY_RIGHT:
            self.cursor_column += y_vel
        elif key == curses.KEY_HOME:
            self.cursor_column -= 10 * y_vel
        elif key == curses.KEY_END:
            self.cursor_column += 10 * y_vel
        elif key == curses.KEY_RESIZE:
            self.resize()
        else:
            handled = False

        if handled:
            self.redraw()
        return handled

    def state_change(self, state):
        if self.state is not None:
            self.state.exit()
        self.state = state
        state.enter()

    def redraw(self):
        # Initialization
        self.stdscr.clear()
        height, width = self.stdscr.getmaxyx()

        # Determine lines to display
        self.put_line_in_view(self.cursor_line)
        display_lines = [ViewLine(*x) for x in enumerate(
            self.filter_lines[self.view_min:self.view_max])]

        # Left Header
        max_line = display_lines[-1].log_line
        left_reserve = len(str(max_line))

        # Bound column
        longest_display = max(len(line.text) for line in display_lines)
        self.cursor_column = max(0, self.cursor_column)
        self.cursor_column = min(longest_display - 2, self.cursor_column)

        # Display Log file
        for line in display_lines:
            bg = curses.COLOR_MAGENTA if line.filter_line == self.cursor_line\
                else curses.COLOR_BLACK
            color = self.get_color(line.level, bg)
            left = '{:>{x}}'.format(line.log_line, x=left_reserve)
            end_column = self.cursor_column + width - (len(left) + 2)
            visible_line = line.text[self.cursor_column:end_column]
            display = '{}  {}'.format(left, visible_line)
            try:
                self.stdscr.addstr(line.view_line, 0, display, color)
            except Exception as e:
                # print_errors.append('Cannot display: {} {}'.format(line.filter_line, display))
                self.stdscr.addstr(0, 0, str(e)[:width], color)

        self.state.status_bar_render()

        # Refresh the screen
        self.stdscr.refresh()

    def resize(self):
        self.view_min = 0
        self.view_max = self.height - 1

    def run(self):
        self.state_change(self.state_map['NORMAL'](self))
        self.resize()
        self.redraw()
        while not self.exit:
            # Wait for next input
            self.key_press(self.stdscr.getch())


class BaseState(object):
    def __init__(self, log_view):
        self.log_view = log_view

    def enter(self):
        pass

    def exit(self):
        pass

    @abstractmethod
    def handle_key(self, key):
        raise NotImplementedError()

    @property
    def stdscr(self):
        return self.log_view.stdscr


class NormalState(BaseState):
    def enter(self):
        self.redraw()

    def status_bar_render(self):
        len_filter = len(self.log_view.filter_lines)
        line_status = 'TOP' if self.log_view.cursor_line == 0 else\
            'BOT' if self.log_view.cursor_line == len_filter-1 else\
            str(self.log_view.cursor_line)
        statusbarstr = "Press 'q' to exit | STATUS BAR | Pos: {} Col: {} " \
                       "View: {}:{} Height: {} FL: {}" \
            .format(line_status, self.log_view.cursor_column,
                    self.log_view.view_min, self.log_view.view_max,
                    self.log_view.height, len(self.log_view.filter_lines))

        height, width = self.stdscr.getmaxyx()
        self.stdscr.attron(self.log_view.get_color(
            curses.COLOR_BLACK, curses.COLOR_CYAN))
        self.stdscr.addstr(height - 1, 0, statusbarstr)
        self.stdscr.addstr(height - 1, len(statusbarstr),
                           " " * (width - len(statusbarstr) - 1))
        self.stdscr.attroff(self.log_view.get_color(
            curses.COLOR_BLACK, curses.COLOR_CYAN))

    def handle_key(self, key):
        handled = True
        if key == ord('q'):
            self.log_view.exit = True
        else:
            handled = False
        return handled

    def redraw(self):
        pass


class GotoState(BaseState):
    @property
    def status_bar_height(self):
        return 1

    def status_bar_render(self):
        pass

    def handle_key(self, key):
        pass


def log_view(stdscr, filename):
    """Main function. Displays a log viewer with Python curses."""
    view = LogView(stdscr, filename)
    view.run()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Log Viewer for Pyliner'
    )
    parser.add_argument('filename', nargs='?',
                        help='Filename of log to view.')

    args = parser.parse_args()
    filename = args.filename

    if filename is None:
        log_files = glob.glob('logs/*')
        filename = max(log_files, key=os.path.getctime)

    if not os.path.isfile(filename):
        print('filename must exist.')
        exit(1)

    if query_yes_no('NOTE: The Pyliner Log Viewer is still in the experimental '
                    'phase.\nBeware of bugs. Continue? '):
        curses.wrapper(log_view, filename)
        print('\n'.join(print_errors))
    else:
        exit()
