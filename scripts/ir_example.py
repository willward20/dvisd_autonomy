import time
from dvisd_autonomy.hardware.ir import IR

def main():
    
    ir_sensor = IR(21)    

    while True:
        state = ir_sensor.get_ir_state()
        print(state)
        time.sleep(0.1)

if __name__ == "__main__":

    main()
