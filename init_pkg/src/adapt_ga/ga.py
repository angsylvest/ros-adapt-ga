import random 

# global var (that can be fine-tuned)
_MAXIMUM_SPEED = 1 # 0.3
_MINIMUM_SPEED = 1 # 0.3 
_COST = 5
_REWARD = 30 
_OBSERV_THRES = 5

random.seed(10)

gene_list = [f'control speed {_MAXIMUM_SPEED}', f'energy cost {_COST}', f'food energy {_REWARD}', f'observations thres {_OBSERV_THRES}']

class GA():
    def __init__(self):
        self.curr_genotype = self.create_individual_genotype() # initially nothing until created  
        self.fitness = 0 
        self.best_encountered_ruleset = ""
        self.best_encountered_fitness = 0

    """
    output: creates random chromosome for that robot 
    """
    def create_individual_genotype(self):
        ruleset = ''
        for gene in gene_list:
            g_count = int(gene.split(" ")[-1])
            ruleset += self.generate_random_act(g_count) + "*"

        self.curr_genotype = ruleset
        return ruleset

    def generate_random_act(self, count): 
        list_binary = ''.join([str(random.randint(0,1)) for i in range(count)])
        # np_binary = np.random.randint(2, size = length)
        # list_binary = ''.join([",".join(item) for item in str(np_binary)])
        return list_binary
    
    """ 
    Important: should update after each collection or each collision (to ensure that this param is up to date)
    """
    def update_fitness(self, num_collected, num_collisions):
        obj_weight = 2 
        obst_weight = 1 

        if num_collisions != 0: 
            return obj_weight*(num_collected) + obst_weight*(1 / num_collisions) # + 0.5*agent_observation['num_objects_observed']
        else: 
            return obj_weight*(num_collected) # + 0.5*agent_observation['num_objects_observed']
        
    # what happens with initial encounter
    def update_best_encountered(self, r2, fitness): # r2 can be ruleset or actually object
        partner_fitness = fitness
        if isinstance(r2, GA):
            ruleset = r2.curr_genotype
        elif isinstance(r2, list):
            rulset = r2 
        else: 
            print('Try again, not expecting that input')


        if partner_fitness > self.best_encountered_fitness:
            self.best_encountered_fitness = partner_fitness
            self.best_encountered_ruleset = rulset
        
        return 
        
    """
    input: original chromosome of curr robot 
    r2 is the ga object for the other partner (or just feed in ruleset)

    when this happens: at the end of a "generation" which happens in 30 sec intervals during foraging task 
    """

    def reproduce(self): 
        global other_genotype 
        # if pass a certain threshold, will increment/decrement randomly by 
        # smaller amount 
        r1 = self.curr_genotype

        if self.best_encountered_ruleset != "" and self.best_encountered_fitness > self.fitness: 
            r2 = self.best_encountered_ruleset
            # if below, resample again 
            new_genotype = ''
            other_genotype = ''
            mom = r1.split("*")
            dad = r2.split("*")
            
            for i in range(len(mom)-1): # assuming this is a list of genotypes 
                child = self.crossover(mom[i], dad[i]) 
                child = self.mutate(child, 0.2) + "*"
                new_genotype += child 
    
            return new_genotype 
        
        else: 
            return r1 # do not update unless partner fitness is better 
    

    def crossover(self, m, d): # d is the higher fitness 
        global other_genotype 
        
        new_child = ''
        other_child = '' 
        
        random_start = random.randint((len(m)-1) // 2,len(m)-1) # always more of d
        new_child += m[:random_start] + d[random_start:] 
        
        other_child += m[random_start:] + d[:random_start] 
        other_child = self.mutate(other_child, 0.2) + "*"
        other_genotype += other_child
        
        return new_child 

    def mutate(self, c, mut_prob):
        # print('child c', c, len(c)) 

        size = len(c) 
        for i in range(1): # make so change only one instead of each item in sequence 
            if random.random() < mut_prob: 
                if i == 0: 
                    # choose random position 
                    p = random.randint(0,int(len(c)/2))
                    # switch to different val 
                    if c[p] == str(1):
                        c = c[:p] + "0" + c[p+1:]
                    else: 
                        c = c[:p] + "1" + c[p+1:]
                
                
                elif i == 1: 
                    # choose random position 
                    p = random.randint(int(len(c)/2), len(c)-1)
                    # switch to different val 
                    if c[p] == str(1):
                        c = c[:p] + "0" + c[p+1:] 
                    else: 
                        c = c[:p] + "1" + c[p+1:] 
                
                else: 
                    pass 
                
        return c  
    
    def process_chromosome(self, chr): # assuming this is in rulset format 
        gen = chr.split("*")
        forward_speed = gen[0].count('1')
        if forward_speed < _MINIMUM_SPEED: 
            forward_speed = _MINIMUM_SPEED

        energy_cost = gen[1].count('1')
        energy_per_item = gen[2].count('1')
        observations_threshold = gen[3].count('1')

        return forward_speed, energy_cost, energy_per_item, observations_threshold

    

""" ---------------------------------
Example Usage: 
1. Initialize 
ga1 = GA()
ga1.create_individual_genotype()


ga2 = GA()
ga2.create_individual_genotype()

2. Process chromosome  
speed, penalty, reward, obs_thrs = ga.process_chromosome(chr) (chr can be any ruleset for instance the child fro ga or original)

3. If encounter, use update_best_encountered() function to update best_encountered so far instance variable

4. At end of generation, then would update using the reproduce() method 

 --------------------------------- """

def main():
    ga_rob = GA()
    ga_rob.create_individual_genotype()

    print(f'example genotpye generated: {ga_rob.curr_genotype}')


    ga_rob_2 = GA()
    ga_rob_2.create_individual_genotype()

    print(f'other example genotpye generated: {ga_rob_2.curr_genotype}')

    processed_chromosome = ga_rob.process_chromosome(ga_rob.curr_genotype)
    print(f'example of processed chromosome: {processed_chromosome}')

    # if encounter, update best_encountered 
    ga_rob.update_best_encountered(r2=ga_rob_2, fitness = 0)


    # reproduce with best at the end of generation 
    new_chr = ga_rob.reproduce()
    print(f'updated chromosome here: {new_chr}')

main()