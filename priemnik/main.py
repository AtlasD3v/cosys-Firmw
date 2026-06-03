from upravlenie import calibrate as cal
from pid_reg import control_loop as cont_loop

from  pid_reg import Control as control
from filters import ckf_sanity_test

def main():

    print("---СИСТЕМА ЗАПУЩЕНА!---\n")


    control_loop = control.Control_loop()
    control_loop.control_loop_func()









if __name__ == '__main__':
    main()