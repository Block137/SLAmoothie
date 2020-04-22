# Gcode formatter by Sunny

import sys
import re
import os
import time


def stringify(gcode):
    return ' '.join([param+value for param, value in gcode])


def parse_gcode(raw):
    # parse gcode as a list of params, where each param is a tuple of (param, value) e.g. (X, 100)
    return [(x[0], x[1:]) for x in raw.strip().split()]


def format_value(value):
    # if value represents float it will throw ValueError, otherwise it is int
    try:
        int(value)
        return str(value)
    except ValueError:
        return str(round(float(value), 2))

# if len(sys.argv) <= 1:
#     print('Please specify filename')
#     exit()

# filename = sys.argv[1]


filenames = [os.path.splitext(file)[0] for file in os.listdir() if file.endswith(
    '.gcode') and not file.endswith('_EDITED.gcode')]
if len(filenames) == 0:
    print('.gcode file not found. Please place the .gcode file in the same folder with this script')
    exit()

for filename in filenames:
    print('Opening {}...'.format(filename))
    time1 = time.time()
    with open(filename + '.gcode') as f:
        with open(filename + '_EDITED.gcode', 'w') as out:

            last_F = [0, 0, 0, 0]
            for line in f:

                # line_edited = re.sub(r'\sE\S+', '', line)
                # m = re.match(r'(G\w+) ([^F]*)(F\S+)(.*)', line)

                if line.startswith(';'):  # if line is comment
                    continue

                if line.startswith('G'):  # if line is gcode

                    gcode = parse_gcode(line)
                    new_gcode = []
                    sep_gcode = [gcode[0]]
                    separate = False

                    # current mode
                    curr_mode = int(gcode[0][1])

                    # For each parameter in this gcode
                    for param, value in gcode:

                        # value = re.sub(r'([^\.]+)\.(\d\d)\d*', r'\1.\2', value)
                        value = format_value(value)

                        # skip E
                        if param == 'E':
                            pass
                        # always separate Z to new line
                        elif param == 'Z':
                            separate = True
                            sep_gcode.append((param, value))
                        # separate F if the value changed, else just skip F
                        elif param == 'F':
                            if curr_mode < len(last_F) and value != last_F[curr_mode]:
                                last_F[curr_mode] = value
                                separate = True
                                sep_gcode.append((param, value))
                        # other parameters are unaffected
                        else:
                            new_gcode.append((param, value))

                    # append separate line before current line
                    if separate:
                        out.write(stringify(sep_gcode) + '\n')

                    out.write(stringify(new_gcode) + '\n')
                else:
                    out.write(line)
    print('Done! ({:.4f} s)'.format(time.time() - time1))

print('Press any key to continue...')
input()
