from cpppo.server.enip.get_attribute import proxy_simple
import logging


class fanuc_robot(proxy_simple):
    PARAMETERS = dict(proxy_simple.PARAMETERS,
                      reg1 = proxy_simple.parameter('@0x04/0x97/0x03/0x0E', 'INT', 'Nm'),
    )

via = fanuc_robot(host="192.168.0.1")

try:
    
    newval = 0
    params = 'reg1 = (INT)%s' % (newval)
    value, = via.write(
        via.parameter_substitution(params), checking=True)
    print(value)
except Exception as exc:
    logging.warning("Access to robot failed: %s", exc)
    via.close_gateway(exc=exc)
    raise
