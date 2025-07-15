#### WORKING READ CODE ####

from cpppo.server.enip.get_attribute import proxy_simple
import logging

class fanuc_robot(proxy_simple):
    PARAMETERS = dict(proxy_simple.PARAMETERS,
                      reg1 = proxy_simple.parameter('@0x6B/1/1', 'DINT', 'Nm'),
    )

via = fanuc_robot(host="192.168.0.1")
try:
    params = via.parameter_substitution("reg1")
    value, = via.read(params, checking=True)
    print(value)
except Exception as e:
    logging.warning("Access to robot failed: %s", e)
    via.close_gateway(e=e)
    raise
