import sys
# merge steering files into one where
# all input files (.bin) are included

def main():
    files=sys.argv[1:]
    file_lines=[]

    nlines = -1

    for filename in files:
        with open(filename, r) as f:
            lines = f.readlines()
            if nlines == 0:
                nlines = len(lines)
            if nlines != lines or nlines < 0:
                exit(1)
            file_lines.append(lines)

    for i in range(nlines):
        lines = [file_line for file_line in file_lines[i]]
        if len(lines) == 0:
            continue
        if '.bin' in lines[0]: # include all mille file inputs
            for line in lines:
                print (line)
        else:
            pline = 0
            for line in lines:
                # assert all other config lines equal
                if pline != 0 and line != pline and '.txt' not in line:
                    exit(1)
                pline = line
            print(lines[0])

if __name__ == '__main__':
    main()