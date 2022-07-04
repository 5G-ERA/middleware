# This script is used for the needs of the middleware to flatten the yaml file so it can be saved in the
# Redis in JSON to be used for the deployment by the middleware
import sys
import getopt


def flatten(inputFile: str, outputFile: str):
    """Takes the input from the input file and saves it in the output file as a single line with according"""

    newLines = []
    with open(inputFile, "r") as f:
        lines = f.readlines()
    with open(f'{outputFile}.fl', "w") as f:
        for line in lines:
            newLines.append(line.replace("\n", "\\n"))
        sentence = "".join(newLines)
        f.write(sentence)
        print(sentence)


def main(argv):
    """Entry point of the application, parses the arguments"""
    inputFile = ""
    outputFile = ""
    try:
        opts, args = getopt.getopt(argv, "hi:o:", ["ifile=", "ofile="])
    except getopt.GetoptError:
        print("test.py -i <inputfile> -o <outputfile>")
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print("test.py -i <inputfile>.yaml -o <outputfile>.txt")
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputFile = arg
        elif opt in ("-o", "--ofile"):
            outputFile = arg
    print('Input file is "', inputFile)
    print('Output file is "', outputFile)

    flatten(inputFile, outputFile)


if __name__ == "__main__":
    main(sys.argv[1:])
