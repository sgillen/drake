#!/usr/bin/python

import re
from common_scripts.text_processor import TextProcessor

class Custom(TextProcessor):
    def process(self, old):
        def rep(m):
            old = m.group(0)
            lines = old.split("\n")
            assert len(lines) >= 3
            first, middle, last = lines[0], lines[1:-1], lines[-1]
            # Determine doc prefix
            suffix = "@endcode"
            assert last.endswith(suffix), repr((last, suffix))
            prefix = last[:-len(suffix)]
            code_lines = []
            code_prefix = None
            for line in middle:
                assert line.startswith(prefix.rstrip()), line
                code_line = line[len(prefix):]
                code_lines.append(code_line)
                ws = re.match(r"^ *", code_line).group(0)
                if code_prefix is None or len(ws) < len(code_prefix):
                    code_prefix = ws
            assert code_prefix is not None
            out = [first]
            for code_line in code_lines:
                line = prefix + code_line[len(code_prefix):]
                out.append(line.rstrip())
            new = "\n".join(out + [last])
            if new != old:
                print(old)
                print("---")
                print(new)
                print("\n\n")
            return "\n".join(out)
        new = re.sub(r"@code.*?@endcode", rep, old, flags = re.M | re.DOTALL)
        return new

Custom().main()
