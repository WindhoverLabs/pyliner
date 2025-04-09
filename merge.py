import argparse
from copy import deepcopy
import json
from os.path import exists,join
import sys

def merge_dicts(airliner, ops, msgs):
    merged = airliner.copy()
    
    # Merge in app msgs first
    for app in merged["Airliner"]["apps"]:
        if app in msgs["Airliner"]["apps"]:
            merged["Airliner"]["apps"][app]["operations"] = msgs["Airliner"]["apps"][app]["operations"]
        else:
            print "Warning: {} not found in msgs json".format(app)

    # Merge operational names for fields
    for app in merged["Airliner"]["apps"]:
        if app in ops["Airliner"]["apps"]:
            merged["Airliner"]["apps"][app]["app_ops_name"] = ops["Airliner"]["apps"][app]["app_ops_name"]
            for sym, sym_data in merged["Airliner"]["apps"][app]["proto_msgs"].iteritems():
                sym_data["operational_names"] = ops["Airliner"]["apps"][app]["proto_msgs"][sym]["operational_names"]
        else:
            print "Warning: {} not found in ops json".format(app)
    

    with open("merged.json", "w") as merged_json:
        json.dump(merged, merged_json, indent=4)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pyliner-json', default='airliner.json',
                        help='The json used by pyliner operations')
    parser.add_argument('--ops', default='pyliner_ops_names.json',
                        help='Json with defined operational names')
    parser.add_argument('--msgs', default='pyliner_msgs.json',
                        help='Json with MsgIds and CmdCodes')
    args = parser.parse_args()

    if exists(args.pyliner_json) and exists(args.ops) and exists(args.msgs):
        with open(args.pyliner_json, "r") as pyliner_json:
            airliner = json.load(pyliner_json)
        with open(args.ops, "r") as ops_json:
            ops = json.load(ops_json)
        with open(args.msgs, "r") as msgs_json:
            msgs = json.load(msgs_json)

        merge_dicts(airliner, ops, msgs)
    else:
        print "All args must exist"
        sys.exit(1)
    

if __name__ == '__main__':
    main()
