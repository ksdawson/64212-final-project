from setup_simulation import setup_simulation
import time
from manipulation.utils import RenderDiagram

def main():
    # Setup simulation
    diagram, simulator = setup_simulation()
    # RenderDiagram(diagram, max_depth=1)

if __name__ == '__main__':
    main()
    time.sleep(15) # makes meshcat actually show