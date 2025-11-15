from setup_simulation import setup_simulation
import time

def main():
    # Setup simulation
    diagram, simulator = setup_simulation()

if __name__ == '__main__':
    main()
    time.sleep(30) # makes meshcat actually show