try:
    import moonbot.utilities.params as params
except:
    import params as params

def changevalue2deg(value, INIT_POS):
    return (INIT_POS - value) * 90 / params.DIFF_ANGLE


def changedeg2value(deg, INIT_POS):
    return int(INIT_POS - params.DIFF_ANGLE/90 * deg)