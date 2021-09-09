file = str(input('Enter the name of the file to remove CRs from: '))

with open(file, 'rb+') as f:
    content = f.read()
    f.seek(0)
    f.write(content.replace(b'\r', b''))
    f.truncate()
