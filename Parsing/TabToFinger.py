import xml.etree.ElementTree as ET
import argparse
if __name__ == "__main__":
    parser=argparse.ArgumentParser(description="reads music XML file (from musescore) and outputs a list of tuples")
    parser.add_argument('file',type=str)
    args=parser.parse_args()
    notelist=[]
    note_values={"eighth":1/2,"quarter":1,"half":2,"whole":4,"16th":1/4}
    beat=0
    try:
        file=open(args.file,encoding="utf8")
    except FileNotFoundError:
        print("Error: file does not exist")
    else:
        with file:
            tree=ET.parse(args.file)
            root=tree.getroot()
            part=root[-1]
            tempo=int(part[0].find('direction').find('sound').get('tempo'))
            beats_per_sec=tempo/60
            for measure in part:
                for note in measure.findall('note'):
                    if note.find("rest") != None:
                        continue
                    notetype=note.find('type').text
                    print(notetype)
                    beat_note=note_values[notetype]
                    beat+=beat_note
                    timestamp=beat*1/beats_per_sec
                    for spec in note.iter("technical"):
                        string=int(spec[0].text)
                        fret=int(spec[1].text)
                        notelist.append((string, fret,timestamp))
            print(notelist)