import argparse
import imp
from logging import DEBUG, INFO
from os.path import basename

from pyliner.util import enable_logging

# TODO If going service route, use config file for setup


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('script',
                        help='The Pyliner script that will be executed.')
    parser.add_argument('--log-dir', default='logs/',
                        help='Directory to put log file. If not given, '
                             'defaults to "logs/".')
    parser.add_argument('--log-file',
                        help='File name of log.\nIf not given, a reasonable '
                             'default is chosen.')
    parser.add_argument('--no-log', action='store_true',
                        help='If passed, disable logging.')
    parser.add_argument('-v', '--debug', action='store_true',
                        help='Enable logging for debug messages.')
    args = parser.parse_args()

    if not args.no_log:
        enable_logging(
            log_dir=args.log_dir,
            log_file=args.log_file,
            script=basename(args.script),
            level=DEBUG if args.debug else INFO)

    imp.load_source('script', args.script)


if __name__ == '__main__':
    main()
