import sys

def main():
    print "Hello, World!"
    print "main() has been called!"

if __name__ == '__main__':
    sys.exit("main.py should not be called directly! use roslaunch instead.")
