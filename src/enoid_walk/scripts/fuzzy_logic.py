import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

class FuzzyLogic:
    def __init__(self):
        self.angle = ctrl.Antecedent(np.arange(0, 27, .1), 'angle')
        self.velocity = ctrl.Antecedent(np.arange(0, 12, .1), 'velocity')
        self.step = ctrl.Consequent(np.arange(0, 12.5, .1), 'step')

        self.angle['low'] = fuzz.trimf(self.angle.universe, [-10.8, 0, 10.8])
        self.angle['medium'] = fuzz.trimf(self.angle.universe, [2.7, 13.5, 24.3])
        self.angle['high'] = fuzz.trimf(self.angle.universe, [16.2, 27, 37.8])

        self.velocity['low'] = fuzz.trimf(self.velocity.universe, [-4.8, 0, 4.8])
        self.velocity['medium'] = fuzz.trimf(self.velocity.universe, [1.2, 6, 10.8])
        self.velocity['high'] = fuzz.trimf(self.velocity.universe, [7.2, 12, 16.8])

        self.step['low'] = fuzz.trimf(self.step.universe, [-5, 0, 5])
        self.step['medium'] = fuzz.trimf(self.step.universe, [1.25, 6.25, 11.25])
        self.step['high'] = fuzz.trimf(self.step.universe, [7.5, 12.5, 17.5])

        rule1 = ctrl.Rule(self.angle['low'] & self.velocity['low'], self.step['low'])
        rule2 = ctrl.Rule(self.angle['medium'] & self.velocity['low'], self.step['low'])
        rule3 = ctrl.Rule(self.angle['high'] & self.velocity['low'], self.step['medium'])

        rule4 = ctrl.Rule(self.angle['low'] & self.velocity['medium'], self.step['medium'])
        rule5 = ctrl.Rule(self.angle['medium'] & self.velocity['medium'], self.step['high'])
        rule6 = ctrl.Rule(self.angle['high'] & self.velocity['medium'], self.step['high'])

        rule7 = ctrl.Rule(self.angle['low'] & self.velocity['high'], self.step['high'])
        rule8 = ctrl.Rule(self.angle['medium'] & self.velocity['high'], self.step['high'])
        rule9 = ctrl.Rule(self.angle['high'] & self.velocity['high'], self.step['high'])

        self.step_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9])
        self.steping = ctrl.ControlSystemSimulation(self.step_ctrl)

    def compute(self, angle, velocity):
        self.steping.input['angle'] = angle
        self.steping.input['velocity'] = velocity

        # Crunch the numbers
        self.steping.compute()

        return self.steping.output['step']

    def vizualize(self):

        self.angle.view()
        self.velocity.view()
        self.step.view()
        self.step.view(sim=self.steping)
        plt.pause(100)

def main():
    fz = FuzzyLogic()

    result = fz.compute(0,0)
    print(result)

    fz.vizualize()


if __name__ == "__main__":
    main()