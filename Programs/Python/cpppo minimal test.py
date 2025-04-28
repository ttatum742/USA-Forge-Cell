# Minimal test
from cpppo.server.enip.get_attribute import proxy_simple
with proxy_simple('192.168.1.100') as proxy:
    print(proxy.read_tag('R[1]'))