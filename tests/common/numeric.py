"""
    Generate simple numeric domain language elements
"""
import tarski as tsk


def generate_numeric_instance():
    lang = tsk.language()

    # The sorts
    particle = lang.sort('particle')

    x = lang.function('x', particle, lang.Real)
    y = lang.function('y', particle, lang.Real)
    f = lang.function('f', particle, lang.Real)

    # Particles
    lang.constant(('p{}'.format(k) for k in (1, 2, 3, 4)), particle)

    return lang