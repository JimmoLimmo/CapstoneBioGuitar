import xml.dom.minidom
#dom=xml.dom.minidom.parseString('TheLittleSea.musicxml')
note_list=[]
# def getText(nodelist):
#     rc = []
#     for node in nodelist:
#         print(node.nodeType)
#         print(node.text)
#         if node.nodeType == node.TEXT_NODE:
#             rc.append(node.data)
#     return ''.join(rc)
def handle_song(Song):
    notes=Song.getElementsByTagName("note")
    handle_notes(notes)
def handle_notes(notes):
    for note in notes:
        handle_note(note)

def handle_note(note):
    string=note.getAttribute("string")
    fret=note.getAttribute("fret")
    print(f"got string {string} and fret {fret}")
# def handle_string(string):
#     return getText(string)
# def handle_fret(fret):
#     return getText(fret)

if __name__=="__main__":
    with open("TheLittleSea.musicXML",encoding='utf-8') as file:
        data=file.read()
        handler=xml.dom.minidom.parseString(data)
        handle_song(handler)