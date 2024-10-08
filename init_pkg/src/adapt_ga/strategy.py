import random 
from math import pi 

class Strategy():
    def __init__(self, reward, penalty, obs_thres, step_size): # last 3 params from GA ruleset

        # available orientations (0, 90, 180, 270)

        # create num_strategies
        self.num_strategies = 4 # CRW, Circular (right), Circular (left), Forward Persistence 

        self.weights = [1 for i in range(self.num_strategies)]
        self.collect_counts = [0 for i in range(self.num_strategies)]
        self.curr_energy = reward
        self.penalty = penalty
        self.obs_thres = obs_thres
        self.step_size = step_size

        self.energy_per_item = 10 
        self.total_collected = 0 
        self.curr_strat_index = 0 
        self.time_exploring_with_strat = 0 # should be updated every second (only homing does this not increase)

        self.curr_index = 0 
        self.assigned_orientations = [0 for i in range(4)] # randomly assigned to nothing


    def update_from_chrom(self, processed_chrom):
        # forward_speed, energy_cost, energy_per_item, observations_threshold
        self.step_size = processed_chrom[0] # basically the speed 
        self.penalty = processed_chrom[1]
        self.energy_per_item = processed_chrom[2]
        self.obs_thres = processed_chrom[3]

    def resample(self, curr_dir):
        if self.energy_expenditure() < 0: 
            self.bias_forward()
        
        if self.obs_thres < self.total_collected: 
            self.weights = self.collect_counts
            print(f'obs greater than total collected: {self.obs_thres, self.total_collected}')
        strat = random.choices(['straight','alternating-left','alternating-right', 'true random'], self.weights)[0] 
        self.curr_strat_index = ['straight','alternating-left','alternating-right', 'true random'].index(strat) 
        self.time_exploring_with_strat = 0 
        self.curr_index = 0 

        print(f'current strat index: {self.curr_strat_index}')
        if self.curr_strat_index == 0: 
            self.assigned_orientations = self.create_crw(curr_dir)
        elif self.curr_strat_index == 1: 
            self.assigned_orientations = self.circ_right()
        elif self.curr_strat_index == 2: 
            self.assigned_orientations = self.circ_left()
        else: 
            self.assigned_orientations = self.create_forward(curr_dir)

        return self.assigned_orientations


    def energy_expenditure(self):
        if self.time_exploring_with_strat != 0: 
            return (self.total_collected*self.energy_per_item  - (self.penalty*self.time_exploring_with_strat))
        else: 
            return self.total_collected*self.energy_per_item 

        
    # after collection, weights 
    def collect(self): 
        incr = 0.02 
        self.collect_counts[self.curr_strat_index] += 1
        self.weights[self.curr_strat_index] = self.weights[self.curr_strat_index] + incr
        self.weights = [float(i)/sum(self.weights) for i in self.weights] 

    def bias_forward(self):
        incr = 0.02 

        self.weights[-1] = self.weights[-1] + incr
        self.weights = [float(i)/sum(self.weights) for i in self.weights] 
    

    def create_crw(self, curr_dir):
        orients = []

        for i in range(4): 
            if round(curr_dir,2) == -0.00 or round(curr_dir,2) == 0.00: 
                orients.append(round(random.choice([0,0, pi/2, -pi/2]),2))
            
            elif round(curr_dir,2) == round(pi/2, 2):
                orients.append(round(random.choice([pi, pi/2, pi/2, 0]),2))
            
            elif round(curr_dir,2) == round(-pi/2): 
                orients.append(round(random.choice([pi, 0, -pi/2, -pi/2]),2))
                
            else: 
                orients.append(round(random.choice([pi,pi, pi/2, -pi/2]),2))

        return orients
    
    def create_forward(self, curr_dir): 
        return [curr_dir, curr_dir, curr_dir, curr_dir]
    
    def circ_right(self): 
        return [round(pi/2, 2), 0, round(-pi/2,2), round(pi,2)]

    def circ_left(self): 
        return [round(pi/2,2), round(pi,2), round(-pi/2,2), 0]


"""
Example usage 

1. initialize strategies (using params from GA)
2. After each sec, check to see if homing. If not increment time_exploring_with_strat
    2a. If avoiding, do wall following until not longer view obstacle then proceed with strategy
3. Proceed with entirety of search strategy 
4. After search strategy complete, resample()


"""

def main():
    st = Strategy(reward=10, penalty=2, obs_thres=5, step_size=5)

    # returns set of orientations that robot should move towards given speed 

    for i in range(3): 
        strategy_generated = st.resample(curr_dir=0.00)
        print(f'example stat generated: {strategy_generated}')


     # update energy 
    st.time_exploring_with_strat += 1

main()