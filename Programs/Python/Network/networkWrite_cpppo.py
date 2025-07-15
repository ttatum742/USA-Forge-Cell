###### WORKING WRITE CODE ######

from cpppo.server.enip.get_attribute import proxy_simple
import logging


class fanuc_robot(proxy_simple):
    PARAMETERS = dict(proxy_simple.PARAMETERS,
                      reg1 = proxy_simple.parameter('@0x6C/1/87', 'REAL', 'Nm'),
    )

via = fanuc_robot(host="192.168.0.1")

try:
    
    newval = 500.05
    params = 'reg1 = (REAL)%d' % (newval)
    value, = via.write(
        via.parameter_substitution(params), checking=True)
    print(value)
except Exception as exc:
    logging.warning("Access to robot failed: %s", exc)
    via.close_gateway(exc=exc)
    raise
