# cHeaders - reads the drone's C++ headers, so the app never has its own copy of a packet layout.
#   structs:   every "struct __attribute__((packed)) Name {...}" with its "static_assert(... "Python 'FMT'")"
#   enums:     NAME = value (or counted from 0) inside every enum
#   constants: constexpr NAME = number
#   defaults:  float name = number; (config structs, e.g. SMConfig::maxDistM)
# A change on the drone (messages.h) is picked up at the next start; a struct whose fields do not
# match its Python format stops the app at once (better than reading garbage in flight).

import re
import struct

TYPES = {"uint8_t": 1, "int8_t": 1, "uint16_t": 1, "int16_t": 1, "uint32_t": 1, "int32_t": 1, "float": 1}


def stripComments(text):
    return re.sub(r"/\*.*?\*/", "", re.sub(r"//[^\n]*", "", text), flags=re.S)


def formatItems(fmt):                   # how many values a struct format packs: '<I2B28f' -> 31
    n = 0
    for count, code in re.findall(r"(\d*)([a-zA-Z?])", fmt):
        n += 1 if code in "sp" else int(count or 1)
    return n


class Struct:
    def __init__(self, name, fields, fmt, size):
        self.name, self.fields, self.st = name, fields, struct.Struct(fmt)
        self.size = self.st.size
        if self.size != size or formatItems(fmt) != len(fields):
            raise ValueError(f"{name}: {len(fields)} fields / {size} bytes in C, Python '{fmt}' has "
                             f"{formatItems(fmt)} / {self.size}")

    def unpack(self, data, offset=0):   # -> dict field: value
        return dict(zip(self.fields, self.st.unpack_from(data, offset)))

    def pack(self, *values, **named):   # by position, or by name (missing names: 0)
        if named:
            values = [named.get(f, 0) for f in self.fields]
        return self.st.pack(*values)


class Header:
    def __init__(self, path):
        with open(path, encoding="utf-8") as f:
            raw = f.read()
        text = stripComments(raw)
        self.structs, self.enums, self.constants = {}, {}, {}

        asserts = {m.group(1): (int(m.group(2)), m.group(3)) for m in
                   re.finditer(r"static_assert\(sizeof\((\w+)\)\s*==\s*(\d+),\s*\"Python '([^']+)'\"\)", raw)}
        for m in re.finditer(r"struct\s+__attribute__\(\(packed\)\)\s+(\w+)\s*\{(.*?)\};", text, re.S):
            name, body = m.group(1), m.group(2)
            if name not in asserts:
                continue
            fields = []
            for line in body.split(";"):
                line = line.strip()
                t = line.split(None, 1)
                if len(t) != 2 or t[0] not in TYPES:
                    continue
                for var in t[1].split(","):
                    var = var.strip()
                    a = re.match(r"(\w+)\[(\d+)\]", var)
                    fields += [f"{a.group(1)}{i}" for i in range(int(a.group(2)))] if a else [var]
            self.structs[name] = Struct(name, fields, *asserts[name][::-1])

        for m in re.finditer(r"enum\s+(\w+)\s*(?::\s*\w+)?\s*\{(.*?)\}", text, re.S):
            values, k = {}, 0
            for item in m.group(2).split(","):
                item = item.strip()
                if not item:
                    continue
                v = re.match(r"(\w+)\s*=\s*(\w+)", item)
                if v:
                    k = int(v.group(2), 0) if v.group(2)[0].isdigit() else values[v.group(2)]
                    values[v.group(1)] = k
                else:
                    values[item] = k
                k += 1
            self.enums[m.group(1)] = values

        for m in re.finditer(r"constexpr\s+\w+\s+(\w+)\s*=\s*(\d+)\s*;", text):
            self.constants[m.group(1)] = int(m.group(2))
        self.defaults = {m.group(1): float(m.group(2)) for m in      # config fields: float maxDistM = 1.5f;
                         re.finditer(r"\bfloat\s+(\w+)\s*=\s*(-?[\d.]+)f?\s*;", text)}

    def value(self, name):              # an enum value or a constant, by name (enums searched in order)
        for e in self.enums.values():
            if name in e:
                return e[name]
        return self.constants[name]
