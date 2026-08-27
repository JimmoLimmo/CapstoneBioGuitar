# -*- coding: utf-8 -*-
"""
Created on Mon Mar 20 14:01:40 2023

@author: andre
"""

import guitarpro as gp

class TimedMeasure(object):
    def __init__(self, measure, time, song):
        self.time = time
        self.measure = measure
        self.song = song
        
    def get_duration(self):
        beats_per_measure = self.measure.header.timeSignature.numerator
        return beats_per_measure / self.song.beat_frequency
    
    def yield_timed_beats(self):
        beat_durations = 0
        for beat in self.measure.voices[0].beats:
            timed_beat = TimedBeat(beat, self.time+beat_durations, self)
            beat_durations += timed_beat.get_duration()
            yield timed_beat
            
    def get_measure_number(self):
        return self.measure.header.number

class TimedBeat(object):
    def __init__(self, beat, time, timed_measure):
        self.time = time
        self.beat = beat
        self.timed_measure = timed_measure
        
    def get_duration(self):
        beat_duration_base = float(self.timed_measure.measure.header.timeSignature.denominator.value) / self.timed_measure.song.beat_frequency
        duration = beat_duration_base/self.beat.duration.value * self.beat.duration.tuplet.times / self.beat.duration.tuplet.enters
        if self.beat.duration.isDotted:
            duration *= 1.5
        return duration
            
    def get_string_map(self):
        return {note.string:note.value for note in self.beat.notes}


class GuitarProSong(object):
    def __init__(self, gp_filepath):
        self.song = gp.parse(gp_filepath)
        self.beat_frequency = float(self.song.tempo)/60 #beats / seconds
            
    def get_track_dict(self):
        return {t.name:t for t in self.song.tracks}
        
    def yield_timed_measures(self,track):
        measure_durations = 0 #seconds
        
        _track = self.song.tracks[track] if type(track) is int else self.get_track_dict()[track]
        for measure in _track.measures:
            timed_measure = TimedMeasure(measure, measure_durations, self)
            measure_durations += timed_measure.get_duration()
            yield timed_measure
        