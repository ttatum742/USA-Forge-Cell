# Split a 32-bit integer into two 16-bit parts
high_word = (newval >> 16) & 0xFFFF  # Upper 16 bits
low_word = newval & 0xFFFF           # Lower 16 bits
params = 'reg1_high = (INT)%s, reg1_low = (INT)%s' % (high_word, low_word)