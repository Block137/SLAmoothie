# Gcode formatter by Sunny
# V1.1

import os
import time

def stringify(gcode):
    return ' '.join([param + value for param, value in gcode])
    # return ' '.join([param + value for param, value in gcode])


def parse_gcode(raw):
    # parse gcode as a list of params, where each param is a tuple of (param, value) e.g. (X, 100)
    return [(x[0], x[1:]) for x in raw.strip().split()]

print('----------------------')
print('Gcode formatter by Sunny')
print('----------------------')

filenames = [os.path.splitext(file)[0] for file in os.listdir() if file.endswith(
    '.gcode') and not file.endswith('.edited.gcode')]
if len(filenames) == 0:
    print('.gcode file not found. Please place the .gcode file in the same folder with this script')
    exit()

for filename in filenames:
    print('Opening {}...'.format(filename))
    time1 = time.time()
    with open(filename + '.gcode') as f:
        with open(filename + '.edited.gcode', 'w', encoding='ascii') as out:

            last_F = [0, 0, 0, 0]
            last_X = 100000  # magic number
            last_Y = 100000

            for line in f:
                if line.startswith(';'):  # if line is comment
                    continue

                if line.startswith('G'):  # if line is gcode
                    gcode = parse_gcode(line)
                    new_gcode = [gcode[0]]
                    sep_gcode = [gcode[0]]
                    curr_mode = int(gcode[0][1])
                    separate = False

                    # For each parameter in this gcode
                    for param, value in gcode[1:]:
                        value = str(round(float(value), 2))
                        if value.endswith('.0'):    # remove trailing zero
                            value = value[:-2]

                        # if X,Y value doesn't change, skip
                        if param == 'X':
                            if value == last_X:
                                continue
                            last_X = value
                        elif param == 'Y':
                            if value == last_Y:
                                continue
                            last_Y = value
                        # skip E
                        elif param == 'E':
                            continue
                        # always separate Z to new line
                        elif param == 'Z':
                            separate = True
                            sep_gcode.append((param, value))
                            continue
                        # separate F if the value changed, else just skip F
                        elif param == 'F':
                            if value != last_F[curr_mode]:
                                last_F[curr_mode] = value
                                separate = True
                                sep_gcode.append((param, value))
                            continue
                        # other parameters are appended as normal
                        new_gcode.append((param, value))

                    # append separate line before current line
                    if separate:
                        out.write(stringify(sep_gcode) + '\n')
                    # append current line only if it has some params after
                    if len(new_gcode) > 1:
                        out.write(stringify(new_gcode) + '\n')
                else:
                    out.write(line)

    print('Done! ({:.4f} s)\n'.format(time.time() - time1))

print('Press any key to continue...')
input()
