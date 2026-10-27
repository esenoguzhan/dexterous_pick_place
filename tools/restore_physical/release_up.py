"""release_up.py: open the hand (thumb first) where it is, then go to the observe pose."""
import restore as rs

t = rs.Restorer()
try:
    t.release()
    t.observe()
finally:
    t.close()
