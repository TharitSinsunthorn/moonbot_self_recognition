try:
    import params as params
except:
    import moonbot.utilities.params as params

DIFF_ANGLE = 1023
def changevalue2deg(value, INIT_POS):
    return (INIT_POS - value) * 90 / DIFF_ANGLE


def changedeg2value(deg, INIT_POS):
    return int(INIT_POS - DIFF_ANGLE/90 * deg)