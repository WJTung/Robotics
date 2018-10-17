from gtts import gTTS

student_name = dict()
student_name['25EDBB5F'] = 'wen chieh'

for id in student_name.keys() :
    message = 'Hello' + ',' + student_name[id]
    tts = gTTS(text = message, lang='en')
    tts.save(id + '.mp3')

message = 'Sorry, unknown UID'
tts = gTTS(text = message, lang='en')
tts.save('unknown.mp3')

message = 'Show me your student ID card!'
tts = gTTS(text = message, lang='en')
tts.save('show_ID' + '.mp3')

message = 'No student ID card?'
tts = gTTS(text = message, lang='en')
tts.save('no_ID' + '.mp3')

message = 'Hello, professor fu'
tts = gTTS(text = message, lang='en')
tts.save('professor.mp3')