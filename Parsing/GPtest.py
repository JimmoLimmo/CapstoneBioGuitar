import guitarpro as gp
import GuitarProReader



if __name__=="__main__":
   mysong= GuitarProReader.GuitarProSong("./red-hot-chili-peppers-snow_hey_oh.gp3")
   measures=mysong.yield_timed_measures(1)
   print(list(measures))