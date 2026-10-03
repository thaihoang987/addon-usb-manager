"""Set the prebuilt image only when release automation has built it."""
import re
import sys
from pathlib import Path
config = Path(__file__).resolve().parents[2] / 'usb_manager/config.yaml'
text = config.read_text(encoding='utf-8')
match = re.search(r'^version:\s*"?([^"\n]+)', text, re.M)
if not match:
    raise SystemExit('Missing add-on version')
if sys.argv[1:] == ['version']:
    print(match.group(1).strip())
elif sys.argv[1:] == ['apply', 'stable']:
    image = 'image: "ghcr.io/thaihoang987/addon-usb-manager"'
    if re.search(r'^image:', text, re.M):
        text = re.sub(r'^image:.*$', image, text, flags=re.M)
    else:
        text += image + '\n'
    config.write_text(text, encoding='utf-8', newline='\n')
else:
    raise SystemExit('Usage: set_channel.py version | apply stable')
