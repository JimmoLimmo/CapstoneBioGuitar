# -*- coding: utf-8 -*-
"""
Created on Wed Mar 15 09:49:33 2023

@author: andre
"""
import math
import itertools
import operator
import heapq as hq

POINTER = 1
MIDDLE = 2
RING = 3
PINKY = 4
BARRE = 5

FINGERS = [POINTER, MIDDLE, RING, PINKY, BARRE]
FINGER_NAMES = {POINTER:"Pointer", MIDDLE:"Middle", RING:"Ring", PINKY:"Pinky", BARRE:"Barre"}
PRIMARY_FINGERS = [POINTER, MIDDLE]
SECONDARY_FINGERS = [RING, PINKY]
COMPLEMENTARY_FINGERS = {POINTER:MIDDLE, MIDDLE:POINTER, RING:PINKY, PINKY:RING}

class InstrumentConfig(object):
    
    def __init__(self, fingers, strings, scale_length):
        self.fingers = fingers
        self.strings = strings
        self.scale_length = scale_length #inches
    def SixStringBarreSetup():
        setup = InstrumentConfig(FINGERS,list(range(1,7)),25.5)
        return setup
    
    def get_fret_position(self,fret_number):
        return self.scale_length - (self.scale_length / math.pow(2 , (fret_number/12)))

    def get_fret_transition_distance(self,start_fret, end_fret):
        return abs(self.get_fret_position(start_fret) - self.get_fret_position(end_fret))



class FingeringGeneratorConfig(object):
    def __init__(self, instrument_config):
        self.instrument_config = instrument_config
        self.min_accessible_string =   FingeringGeneratorConfig.get_finger_dict(2,1,1,1,1) #indexed by finger actuator
        self.max_accessible_string =   FingeringGeneratorConfig.get_finger_dict(6,6,6,6,6) #indexed by finger actuator
        self.max_accessible_fret =     FingeringGeneratorConfig.get_finger_dict(20,20,24,24,16)
        self.body_fret_offset =        FingeringGeneratorConfig.get_finger_dict(0,1,0,1,0) #the body of the finger is offset this many frets, blocking notes on adjacent frets
        self.allow_barre_crossing = False
        self.force_primary_low = POINTER
        self.force_primary_spacing = 1
        self.max_adjacent_strings =  FingeringGeneratorConfig.get_finger_dict(2,2,1,1,6)
        self.min_adjacent_strings =  FingeringGeneratorConfig.get_finger_dict(1,1,1,1,6)
        self.finger_restrictions = FingeringGeneratorConfig.get_finger_dict([(MIDDLE,operator.le)], [(POINTER,operator.ge),(PINKY,operator.le)],[(PINKY,operator.le)],[(RING,operator.ge),(MIDDLE,operator.ge)], [(POINTER,operator.lt),(RING,operator.lt)]) #Assert(val[1](a=key,b=val[0]))
        self.idle_fingering = FingeringGeneratorConfig.get_finger_dict(FingerPosition(1), FingerPosition(2), FingerPosition(3), FingerPosition(4),FingerPosition(0))
        self.note_fingering_duration_ratio = 0.8 #how much of note duration is spend with it being fingered
        self.cost_held_unplayed_string = 100
        self.finger_acceleration_cost_multiplier = FingeringGeneratorConfig.get_finger_dict( 2, 2, 1, 1, 1)
        
    def get_finger_dict(pointer,middle,ring,pinky,barre):
        return {POINTER:pointer, MIDDLE:middle, RING:ring, PINKY:pinky, BARRE:barre}

        
#indicates what strings should be fretted and played for a given beat/note
class StringPositions(object):
    def __init__(self, string_map):
        self.string_map = string_map #keys are only those strings being played (#6 -> #1), val is fret of string
        self.distinct_frets = list(set(f for s,f in self.string_map.items()))
        self.fret_map = {fret: [s for s,f in string_map.items() if f == fret] for fret in self.distinct_frets} #keys are only those frets being used (#0->?), val are strings held at fret
    def from_chord_list(chord_list):
        string_map = {len(chord_list)-i:int(p) for i,p in enumerate(chord_list) if p.isnumeric()}
        return StringPositions(string_map)
    def get_tab_string(self, num_strings):
        tab_string = ""
        for s in range(num_strings,0,-1):
            if s in self.string_map:
                tab_string += f"{self.string_map[s]:2} "
            else:
                tab_string += " | "
        return tab_string
    def __repr__(self):
        return self.string_map.__repr__()
    __str__ = __repr__
        
    
class FingerPosition(object):
    def __init__(self, fret, strings = []):
        self.fret = fret
        self.strings = strings
    def __repr__(self):
        return f"({self.fret:2}, {self.strings})"
    __str__ = __repr__
    def get_fingering_string(self, num_strings):
        out = f"{self.fret:2}: "
        for s in range(num_strings,0,-1):
            if s in self.strings:
                out += f"{s}"
            else:
                out += " "
        return out
        
        
class FingeringNode(object):
    def __init__(self,time,duration,fingering,positions):
        self.fingering = fingering
        self.positions = positions
        self.time = time #not duration. this is when the fingering happens
        self.duration = duration
        
        #used in astart search:
        self.previous_node = None
        self.cumulative_cost = 0
        self.positions_index = 0
        
    def get_cost(self):
        prev_cost = self.previous_node.cumulative_cost if self.previous_node is not None else 0
        return self.cumulative_cost - prev_cost
    
        
    def __eq__(self, o):
        if o is None:
            return False
        return o.fingering == self.fingering and o.time == self.time and o.cumulative_cost == self.cumulative_cost and o.duration == self.duration and o.position_index == self.position_index

    def __hash__(self):
        return hash(self.__str__())
    
    def __repr__(self):
        return f"({self.time},{self.positions.__repr__()},{self.fingering.__repr__()})"
    
    __str__ = __repr__
    
    def get_fingering_string(self, num_strings):
        sorted_fingering = list(self.fingering.items())
        sorted_fingering.sort(key=lambda f: (f[1].fret,f[0]))
        out = "/".join(str(f[0]) for f in sorted_fingering)
        for f in self.fingering.items():
            out += f"   |{f[0]}-"+f[1].get_fingering_string(num_strings)  
        return out

    def get_node_cost_string(self, generator):
        cost = generator.get_node_transition_cost(self.previous_node, self) if self.previous_node is not None else 0
        accelerations = generator.get_node_transition_accelerations(self.previous_node, self) if self.previous_node is not None else {0:0}
        return f"t={self.time:6f}: {self.positions.get_tab_string(len(generator.config.instrument_config.strings))}\t{self.get_fingering_string(len(generator.config.instrument_config.strings))}\t${cost:6.0f}\ta={max(accelerations.values())/386:3.2f}g"
    
    def __lt__(self, other):
        return self.cumulative_cost < other.cumulative_cost


class FingeringGenerator(object):
    def __init__(self, config):
        self.config = config

    def __yield_barres_of_fret(self, fret, positions):
        #check for full barre on highest fret
        blocked_strings = [string for string in positions.string_map if positions.string_map[string] < fret]
        if not blocked_strings:
            yield self.config.instrument_config.strings
        #check for 2-barres on any fret
        strings = positions.fret_map[fret]
        for i_string in range(0,len(strings)-1):
            if abs(strings[i_string] - strings[i_string+1]) == 1:
                yield [strings[i_string+1],strings[i_string]]
        #check for possible masked 2-barres. not because they are a better option than just oen finger...but when comign from another note it could be handy
        for string in positions.fret_map[fret]:
            if not((string+1) in positions.string_map and positions.string_map[string+1] <= fret):
                yield [string,(string+1)]
            if not((string-1) in positions.string_map and positions.string_map[string-1] <= fret):
                yield [(string-1),string]
                
    def __yield_finger_position_sets_of_fret(self, fret, positions):
        yield [FingerPosition(fret,[string]) for string in positions.fret_map[fret]] #default case - no barres
        for barre in self.__yield_barres_of_fret(fret, positions):
            yield [FingerPosition(fret,[string]) for string in positions.fret_map[fret] if (string > max(barre) or string < min(barre))] + [FingerPosition(fret, barre)]
            
    def __yield_finger_position_sets_recursive(self,i_fret, positions):
        
        if i_fret >= len(positions.fret_map):
            yield []
            return
        if positions.distinct_frets[i_fret] == 0:
            i_fret = i_fret + 1
        if i_fret >= len(positions.fret_map):
            yield []
            return
        
        #print(f"  Evaluating Fret Groupings on i{i_fret} (#{list(fret_map.keys())[i_fret]})")
        this_fret_finger_position_sets = list(self.__yield_finger_position_sets_of_fret(positions.distinct_frets[i_fret], positions))
        #print(f"    This Fret Groupings{this_fret_groupings}")
        other_fret_finger_position_sets = list(self.__yield_finger_position_sets_recursive(i_fret+1, positions))
        for this_fret_finger_position_set in this_fret_finger_position_sets:
            for other_fret_finger_position_set in other_fret_finger_position_sets:
                #print(f"    YIELD @{i_fret} {this_fret_grouping} AND {partial_fret_grouping}")
                yield this_fret_finger_position_set + other_fret_finger_position_set
                
        
            
    # returns all potential fingerings (only active fingers) of the supplied fret groupings (fret, stringstart, stringend) by putting them in a dict with finger as key
    def __yield_active_fingerings_of_finger_position_set(self,finger_position_set, fingers):
        num_finger_positions = len(finger_position_set)
        if num_finger_positions > len(fingers)-1:
            return
        
        for finger_orders in itertools.permutations(fingers, num_finger_positions):
            yield {finger_orders[i]:finger_position_set[i] for i in range(num_finger_positions)}
            
            
    def __is_valid_fingering(self,fingering):
        
        for finger in fingering:
            #apply finger order constraints
            for restriction in self.config.finger_restrictions[finger]:
                if restriction[0] in fingering and not restriction[1](fingering[finger].fret, fingering[restriction[0]].fret):
                        return False
            
            #fingers can only finger valid groups of adjacent strings
            if fingering[finger].strings:
                if len(fingering[finger].strings) > self.config.max_adjacent_strings[finger]:
                    return False
                if len(fingering[finger].strings) < self.config.min_adjacent_strings[finger]:
                    return False
                
            #apply accessible fret and finger constraints
            if fingering[finger].strings:
                if fingering[finger].fret > self.config.max_accessible_fret[finger]:
                    return False
                if max(fingering[finger].strings) > self.config.max_accessible_string[finger]:
                    return False
                if min(fingering[finger].strings) < self.config.min_accessible_string[finger]:
                    return False

        #check for primary fingers hitting the body of the secondary fingers
        for secondary_finger in SECONDARY_FINGERS:
            if secondary_finger in fingering and fingering[secondary_finger].strings:
                blocked_fret = fingering[secondary_finger].fret+self.config.body_fret_offset[secondary_finger]
                for finger in fingering:
                    if finger != secondary_finger and fingering[finger].strings and fingering[finger].fret == blocked_fret and max(fingering[finger].strings) < min(fingering[secondary_finger].strings):
                        return False
        
        #apply pointer/middle finger strings spacing and ordering constraints
        if POINTER in fingering and MIDDLE in fingering and fingering[POINTER].strings and fingering[MIDDLE].strings and fingering[POINTER].fret == fingering[MIDDLE].fret:
            if self.config.body_fret_offset[POINTER] == self.config.body_fret_offset[MIDDLE]:
                return False
            spacing = 0
            if min(fingering[POINTER].strings) > max(fingering[MIDDLE].strings): #index finger is on a lower string (larger number)
                if self.config.force_primary_low == MIDDLE:
                    return False
                spacing = min(fingering[POINTER].strings) - max(fingering[MIDDLE].strings) - 1
            elif max(fingering[POINTER].strings) < min(fingering[MIDDLE].strings): #index finger is on a higher string
                if self.config.force_primary_low == 1:
                    return False
                spacing = min(fingering[MIDDLE].strings) - max(fingering[POINTER].strings) - 1
            else: #the fingers overlap! probably should have caught this sooner...
                return False
            if spacing < self.config.force_primary_spacing:
                return False
        
        
            
                
        #prevent abrre actuator for crossing under any other fingers, even when disengaged
        if not self.config.allow_barre_crossing and BARRE in fingering:
            for finger in fingering:
                if finger != BARRE and fingering[finger].fret < fingering[BARRE].fret:
                    return False


        return True
    
    
    # generator that returns every possible fingering for the given position given the finger actuator constraints
    def yield_active_fingerings(self,positions,fingers):
        for finger_position_set in self.__yield_finger_position_sets_recursive(0, positions):
            #print(f"  Potential Fret Grouping: {fret_groupings}")
            for fingering in self.__yield_active_fingerings_of_finger_position_set(finger_position_set,fingers):
                #print(f"    Potential Fingering: {fingering}")
                if self.__is_valid_fingering(fingering):
                    yield fingering
                    
    # return all possible frets a finger *could* have been on (because no-string fingering is actualyl quite flexible)
    def __get_accessible_frets(self, finger, fingering):
        if finger in fingering and fingering[finger].strings:
            return [fingering[finger].fret]
        frets = list(range(0,self.config.max_accessible_fret[finger]+1))
        for restriction in self.config.finger_restrictions[finger]:
            if restriction[0] in fingering and fingering[restriction[0]].strings:
                frets = list(f for f in frets if restriction[1](f,fingering[restriction[0]].fret))
        return frets                
    

    
    def __yield_potential_inactive_frets_of_finger(self, finger, active_fingering, reference_fingerings):
        valid_frets = self.__get_accessible_frets(finger, active_fingering)
        reference_frets_of_finger = list(set(reference_fingering[finger].fret for reference_fingering in reference_fingerings if reference_fingering[finger].fret in valid_frets))
        for fret in reference_frets_of_finger:
            yield fret #it was an old position, try leaving it there (although it may not be valid)
            #if finger in COMPLEMENTARY_FINGERS and COMPLEMENTARY_FINGERS[finger] in active_fingering:
                #yield int((active_fingering[COMPLEMENTARY_FINGERS[finger]].fret + fret)/2)
        #if not reference_frets_of_finger:
        #if finger in COMPLEMENTARY_FINGERS and COMPLEMENTARY_FINGERS[finger] in active_fingering:
            #yield active_fingering[COMPLEMENTARY_FINGERS[finger]].fret
        if active_fingering:
            if finger == BARRE:
                yield min(active_fingering.keys())-1
            else:
                for active_finger in active_fingering:
                    yield active_fingering[active_finger].fret
        else:
            yield self.config.idle_fingering[finger].fret
                
        
        
        
    def __yield_finger_positions_of_finger(self,finger,active_fingering, reference_fingerings):
        if finger in active_fingering:
            yield active_fingering[finger]
        else:
            for fret in list(set(self.__yield_potential_inactive_frets_of_finger(finger, active_fingering, reference_fingerings))):
                yield FingerPosition(fret)
                    
    def __yield_finger_positions_of_finger_recursive(self,i_finger, fingers, active_fingering, reference_fingerings):
        for fingering in self.__yield_finger_positions_of_finger(fingers[i_finger], active_fingering, reference_fingerings):
            if i_finger == 0:
                yield {fingers[i_finger]: fingering}
            else:
                for sub_fingering in self.__yield_finger_positions_of_finger_recursive(i_finger-1, fingers, active_fingering, reference_fingerings):
                   sub_fingering_copy = sub_fingering.copy()
                   sub_fingering_copy[fingers[i_finger]] = fingering
                   yield sub_fingering_copy

    
    def yield_full_fingerings(self,string_positions, fingers, reference_fingerings):
        for active_fingering in self.yield_active_fingerings(string_positions, fingers):
            #print(f"Active fingering{active_fingering}")
            for full_fingering in self.__yield_finger_positions_of_finger_recursive(len(fingers)-1,fingers,active_fingering,reference_fingerings):
                #print(f"Potential fingering: {full_fingering}")
                if self.__is_valid_fingering(full_fingering):
                    yield full_fingering
                    



    def __get_smallest_possible_distance(self, finger,start_fingering, end_fingering):
        start_frets = self.__get_accessible_frets(finger,start_fingering) #all possible frets finger could start on
        end_frets = self.__get_accessible_frets(finger,end_fingering) #all possible frets finger could end on
        if max(start_frets) < min(end_frets):#all fo start range less than all of end range
            return self.config.instrument_config.get_fret_transition_distance(max(start_frets), min(end_frets))
        elif max(end_frets) < min(start_frets):#all of end range less than all of start range
            return self.config.instrument_config.get_fret_transition_distance(max(end_frets), min(start_frets))
        return 0
        
    
    def __get_finger_transition_acceleration(self, start_fret, end_fret, time):
        distance = self.config.instrument_config.get_fret_transition_distance(start_fret, end_fret)
        if time <= 0: 
            return float('inf')
        return 2 * distance / (time * time)
    
    def __get_finger_transition_time(self, start_finger_position_strings, start_time, duration, end_time):
        true_start_time = start_time if not start_finger_position_strings else start_time + duration * self.config.note_fingering_duration_ratio
        return abs(end_time - true_start_time)
    
    def __get_node_transition_time_of_finger(self, finger, start_node, end_node):
        return self.__get_finger_transition_time(start_node.fingering[finger].strings, start_node.time, start_node.duration, end_node.time)


    def __get_finger_position_transition_cost(self, finger, start_finger_position, end_finger_position, time):
        acceleration = self.__get_finger_transition_acceleration(start_finger_position.fret, end_finger_position.fret, time)
        return acceleration * self.config.finger_acceleration_cost_multiplier[finger]

    def __get_finger_transition_cost(self, finger, start_fingering, end_fingering, time):
        #distance = self.__get_smallest_possible_distance(finger, start_node.fingering, end_node.fingering)
        if finger not in start_fingering or finger not in end_fingering:
            return 0
        return self.__get_finger_position_transition_cost(finger,start_fingering[finger], end_fingering[finger], time)
        
    #find earliest node where this finger was on the same fret and not playing a string
    def __get_previous_fingered_node(self, node, finger):
        current_node = node
        if finger in current_node.fingering and current_node.fingering[finger].strings:
            return current_node
        while (current_node.previous_node is not None) and (finger in current_node.previous_node.fingering) and not(current_node.previous_node.fingering[finger].strings) and current_node.fingering[finger].fret == current_node.previous_node.fingering[finger].fret:
            current_node = current_node.previous_node
        return current_node
        
    def __get_node_transition_cost_of_finger(self, finger, start_node, end_node):
        previous_fingered_node = self.__get_previous_fingered_node(start_node, finger)
        time = self.__get_node_transition_time_of_finger(finger, previous_fingered_node, end_node)
        return self.__get_finger_transition_cost(finger, start_node.fingering, end_node.fingering, time)
        
    def __get_fingering_cost(self,fingering, positions):
        cost = 0
        for finger in fingering: #prefer not using barre
            held_unplayed_strings = sum(1 for s in fingering[finger].strings if not(s in positions.string_map and fingering[finger].fret == positions.string_map[s]))
            cost += self.config.cost_held_unplayed_string*math.pow(held_unplayed_strings,2)
        return cost
        
    def __get_node_cost(self, node):
        return self.__get_fingering_cost(node.fingering, node.positions)
    
        
    def get_node_transition_cost(self, start_node, end_node):
        #do i just use the max? or average? or sum? or product?
        return self.__get_node_cost(start_node) + self.__get_node_cost(end_node) + sum(self.__get_node_transition_cost_of_finger(finger, start_node, end_node) for finger in FINGERS)

    def get_node_transition_accelerations(self, start_node, end_node):
        start_nodes = {f:self.__get_previous_fingered_node(start_node, f) for f in start_node.fingering.keys()}
        return {finger:self.__get_finger_transition_acceleration(start_nodes[finger].fingering[finger].fret, end_node.fingering[finger].fret, self.__get_finger_transition_time(start_nodes[finger].fingering[finger].strings, start_nodes[finger].time, start_nodes[finger].duration, end_node.time)) for finger in start_node.fingering}


    def __can_transition(self, start_node, end_node):
        #for finger in end_node.fingering:
        #    if not end_node.fingering[finger].strings and (start_node.fingering[finger].fret != end_node.fingering[finger].fret):
        #       return False
        return True
    

    
    def __get_lowest_transition_cost(self, start_positions, end_positions, start_time, duration, end_time):
        min_cost = None
        for start_fingering in self.yield_active_fingerings(start_positions, self.config.instrument_config.fingers):
            start_fingering_cost = self.__get_fingering_cost(start_fingering, start_positions)
            for end_fingering in self.yield_active_fingerings(end_positions, self.config.instrument_config.fingers):
                end_fingering_cost = self.__get_fingering_cost(end_fingering, end_positions)
                transition_cost = sum(self.__get_finger_transition_cost(f,start_fingering, end_fingering,self.__get_finger_transition_time(start_fingering[f] if f in start_fingering else [], start_time, duration, end_time)) for f in end_fingering.keys()) if end_fingering else 0
                test_min_cost = start_fingering_cost + end_fingering_cost + transition_cost
                if min_cost is None or min_cost > test_min_cost:
                    min_cost = test_min_cost
                    #print(f"{test_min_cost:4.0f}: {start_fingering_cost:4.0f}, {transition_cost:4.0f}, {end_fingering_cost:4.0f}")
                if min_cost == 0:
                    
                    break
            if min_cost == 0:
                break
        #print(f"{start_fingering} -> {end_fingering}")
        return min_cost
    
    def get_lowest_transition_cost(self, start_positions, end_positions, start_time, duration, end_time):
        return self.__get_lowest_transition_cost(start_positions, end_positions, start_time, duration, end_time)
                
    
    def get_fingering_sequence_from_timed_position_sequence(self, start_node, position_sequence, times, durations):     
        

        
        #setup 'heuristic': estimated cost to destination from a given node. important not to overshoot because its impossible to update nodes in astar queue
        position_heuristic_lookup = []
        heuristic_cost = 0
        for i in range(len(position_sequence),0,-1):
            if i < len(position_sequence):
                heuristic_cost += self.__get_lowest_transition_cost(position_sequence[i-1], position_sequence[i], times[i-1], durations[i-1], times[i])
            position_heuristic_lookup.insert(0,heuristic_cost)
        #print(position_heuristic_lookup)
        open_nodes = []
        closed_nodes = dict()
        start_node.positions_index = -1
        start_node.cumulative_cost = 0
        end_positions_index = len(position_sequence)-1
        
        hq.heappush(open_nodes, (0,start_node))
        iteration = 0    
        while open_nodes:
            iteration += 1
            _,current_node = hq.heappop(open_nodes)
            if current_node.positions_index == end_positions_index:
                path = []
                while current_node.positions_index >= 0:
                    path.insert(0,current_node)
                    current_node = current_node.previous_node
                #print(f"Found solution after {iteration} iterations")
                return path
            
            closed_nodes[current_node.__repr__()]=current_node
            next_position_index = current_node.positions_index+1
            #generate all potential child nodes at next position
            for fingering in self.yield_full_fingerings(position_sequence[next_position_index], FINGERS, [current_node.fingering]):
                #if next_position_index >= 5:
                    #print(fingering)
                next_node = FingeringNode(times[next_position_index],durations[next_position_index],fingering,position_sequence[next_position_index])
                next_node_transition_cost = self.get_node_transition_cost(current_node, next_node)
                next_node.cumulative_cost = current_node.cumulative_cost + next_node_transition_cost
                if not next_node.__repr__() in closed_nodes or closed_nodes[next_node.__repr__()].cumulative_cost > next_node.cumulative_cost:
                    next_node.previous_node = current_node
                    next_node.positions_index = next_position_index
                    total_path_estimate = position_heuristic_lookup[next_position_index] + next_node.cumulative_cost
                    #print(f"{iteration:6}\t\tDepth={current_node.positions_index:2}\t\t Current Best Cost: {current_node.cumulative_cost:6.0f}\t\tChild Cost: {next_node.cumulative_cost:6.0f}")
                    hq.heappush(open_nodes, (total_path_estimate, next_node))
                    
        print("NO SOLUTION FOUND TO POSITION SEQUENCE")
        
    def get_fingering_sequence_from_timed_beats(self, start_node, timed_beats):
        position_sequence = [StringPositions(tb.get_string_map()) for tb in timed_beats]
        times = [tb.time for tb in timed_beats]
        durations = [tb.get_duration() for tb in timed_beats]
        return self.get_fingering_sequence_from_timed_position_sequence(start_node, position_sequence, times, durations)

    
    
"""    
instrument_config = InstrumentConfig.SixStringBarreSetup()
generator_config = FingeringGeneratorConfig(instrument_config)
fingering_generator = FingeringGenerator(generator_config)
start_positions = StringPositions.from_chord_list(['x','3','3','2','1','1'])
end_positions = StringPositions.from_chord_list(['x','0','1','0','2','3'])
print(fingering_generator.get_lowest_transition_cost(start_positions, end_positions, 0, 0.3, 0.3))


beat_duration = 1 / (180*4/60) #BPM * size_of_beat / to_seconds  <-invert

beats = [
    ['x','9','x','x','x','x'],
    ['x','x','7','x','x','x'],
    ['x','x','x','6','x','x'],
    ['x','x','x','x','7','x'],
    ['x','x','x','x','x','5'],
    ['x','x','x','x','x','9'],
    ['x','x','x','x','x','17'],
    ['x','x','x','x','x','9'],
    ['x','x','x','x','x','5'],
    ['x','x','x','x','7','x'],
    ['x','x','x','6','x','x'],
    ['x','x','7','x','x','x'],
    ['x','10','x','x','x','x'],
    ['x','x','9','x','x','x'],
    ['x','x','x','7','x','x'],
    ['x','x','x','x','8','x'],
    ['x','x','x','x','x','7'],
    ['x','x','x','x','x','10'],
    ['x','x','x','x','x','15'],
    ['x','x','x','x','x','10'],
    ['x','x','x','x','x','7'],
    ['x','x','x','x','8','x'],
    ['x','x','x','7','x','x'],
    ['x','x','9','x','x','x']]

beat_positions = [StringPositions.from_chord_list(b) for b in beats]
start_node = FingeringNode(0,beat_duration*4,generator_config.idle_fingering,StringPositions({}))

fingering_sequence = fingering_generator.get_fingering_sequence_from_position_sequence(start_node, beat_positions, beat_duration)


print(start_node.get_node_cost_string(fingering_generator))
for i,node in enumerate(fingering_sequence):
    print(node.get_node_cost_string(fingering_generator))
print(f"cost: {fingering_sequence[-1].cumulative_cost}")
"""