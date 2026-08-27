# -*- coding: utf-8 -*-
"""
Created on Mon Mar 20 15:00:20 2023

@author: andre
"""

from Fingering import *
from GuitarProReader import *
from collections import deque
import csv


MAX_BEATS_PER_SOLVE = 4


def yield_song_fingering(gp_file, generator):
    song = GuitarProSong(gp_file)
    
    start_node = FingeringNode(-1,1,generator.config.idle_fingering,StringPositions({}))
    
    solving_group = list()
    reference_group = list()
    beat_group_queue = deque()
    
    for timed_measure in song.yield_timed_measures(0):
        print(f"Evaluating Measure #{timed_measure.measure.header.number}")
        beats_in_measure = [beat for beat in timed_measure.yield_timed_beats()]
        groups = int(len(beats_in_measure)/MAX_BEATS_PER_SOLVE)+1
        group_size = int(len(beats_in_measure)/groups)+1
        for g in range(0,groups):
            beat_group = [beats_in_measure[b] for b in range(g*group_size,min(len(beats_in_measure),(g+1)*group_size))]
            beat_group_queue.append(beat_group)
        
        while beat_group_queue:
            solving_group = reference_group
            reference_group = beat_group_queue.popleft()
            if solving_group:
                sequence = generator.get_fingering_sequence_from_timed_beats(start_node, solving_group+reference_group)
                for i in range(0,len(solving_group)):
                    print(sequence[i].get_node_cost_string(generator))
                    yield sequence[i]
                start_node = sequence[len(solving_group)-1]
                sequence[0].previous_node = None
                
    for i in range(len(solving_group), len(sequence)):
        print(sequence[i].get_node_cost_string(generator))
        yield sequence[i]
        
       
        
def yield_measure_fingering(measure, generator, start_node):
    print(f"Evaluating Measure #{measure.header.number}")
    durations = list()
    positions = list()
    for i,(_, duration, string_map) in enumerate(song.yield_timed_string_positions(start_node.time, measure)):
        if i > -1:
            durations.append(duration)
            positions.append(StringPositions(string_map))
    

    sequence = generator.get_fingering_sequence_from_position_sequence(start_node, positions,durations)
    for i in range(0,len(sequence)):
        print(sequence[i].get_node_cost_string(fingering_generator))
        yield sequence[i]


    
    
        
        
instrument_config = InstrumentConfig.SixStringBarreSetup()
generator_config = FingeringGeneratorConfig(instrument_config)
fingering_generator = FingeringGenerator(generator_config)        
song_file = "Parsing/led-zeppelin-whole_lotta_love_2.gp3"   
#song_file = "Songs\Lynyrd Skynyrd - Sweet Home Alabama.gp3"
#out_file = open("Fingering - Sweet Home Alabama.txt",'w')
#song_file = "Songs\Racer X - Technical Difficulties.gp3"
#out_file = open("Fingering - Technical Difficulties.txt",'w')


#song = GuitarProSong(song_file)
#measure = song.song.tracks[0].measures[36]
#start_node = FingeringNode(-1,1,generator_config.idle_fingering,StringPositions({}))

#for node in yield_measure_fingering(measure, fingering_generator, start_node):

def node_to_csv_rows(node):
    string_to_finger = {}
    rows = []
    for finger, finger_pos in node.fingering.items():
        for string in finger_pos.strings:
            string_to_finger[string] = finger
    
    for string, fret in node.positions.string_map.items():
        finger = string_to_finger.get(string, None)
        rows.append([f"{node.time:.6f}", node.duration, string, fret, finger,])
    return rows

with open("Parsing/parsed.csv", 'w', newline='', encoding='utf-8') as f:
    writer = csv.writer(f)
    writer.writerow(["time", "duration", "string", "fret", "finger"])
    for item in yield_song_fingering(song_file, fingering_generator):
        for row in node_to_csv_rows(item):
            writer.writerow(row)