"""
To run:

    ls *.ipynb | xargs -t -n 1 python3 ./strip_metadata.py
"""
import sys
import json

_, fname = sys.argv
with open(fname) as f:
    doc = json.load(f)
for cell in doc["cells"]:
    if "metadata" in cell:
        cell["metadata"] = {}
with open(fname, "w") as f:
    f.write(json.dumps(doc, indent=True) + "\n")
