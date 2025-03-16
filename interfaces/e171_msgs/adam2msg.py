import os
import glob

home = "../proxy/include/data_interface"

files = glob.glob(os.path.join(home, "*.h"))

def type_mapping(in_type):
    if in_type == 'uint16_t':
        return 'uint16'
    elif in_type == 'int16_t':
        return 'int16'
    elif in_type == 'uint8_t':
        return 'uint8'
    elif in_type == 'int8_t':
        return 'int8'
    elif in_type == 'float':
        return 'float32'
    elif in_type == 'double':
        return 'float64'
    elif in_type == 'int':
        return 'int32'
    elif in_type == 'int32_t':
        return 'int32'
    elif in_type == 'uint32_t':
        return 'uint32'
    elif in_type == 'uint64_t':
        return 'uint64'
    elif in_type == 'bool':
        return 'bool'
    else:
        in_type = in_type[0].upper() + in_type[1:]
        print('unknown type', in_type)
        return in_type

for f in files:
    print(f)
    start = False
    msgname = os.path.basename(f).replace('_', '')[:-2]
    msgname = msgname[0].upper() + msgname[1:]
    if msgname.endswith("Enum"):
        continue
    filename = os.path.join('msg',  msgname + ".msg")
    fin = open(f, 'r')
    fout = open(filename, 'w')
    lines = fin.readlines()
    for line in lines:
        if line.strip().startswith('typedef struct'):
            # enter msg define
            start = True
            continue
        if line.strip().startswith('}'):
            start = False
            break
        if start is True:
            try:
                t, name = line.strip().split(' ')
            except:
                from IPython import embed;embed()
            name = name.replace(';', '')
            name = name.lower()
            mapped_type = type_mapping(t)
            mapped_type = mapped_type.replace('_', '')
            if name.endswith(']'):
                for i in range(len(name)):
                    if name[i] == '[':
                        match = i
                        break
                mapped_type += name[match:]
                name = name[:match]
            print(mapped_type, name, file=fout)



