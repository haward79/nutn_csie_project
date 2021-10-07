
from datetime import datetime


def logText(msg: str):

    now = datetime.now().strftime('[%Y.%m.%d %I:%M:%S] ')

    print(now + msg)

    with open('text.log', 'a') as fout:
        fout.write(now + msg + '\n')

