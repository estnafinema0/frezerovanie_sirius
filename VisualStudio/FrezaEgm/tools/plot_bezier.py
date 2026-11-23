"""Reproduce the offline crown illustration; Python stdlib, not an application dependency."""
import csv
from pathlib import Path
root = Path(__file__).resolve().parents[3]
points = [tuple(float(r[k]) for k in ('x','y','z')) for r in csv.DictReader((root/'VisualStudio/FrezaEgm/data/bezier/crown-part.csv').open())]
def height(x,y):
    u,v=x/20,y/20
    return 8*u*(1-u)+4*v*(1-v)
def project(p):
    x,y,z=p
    return 440+16*(x-y),160+7*(x+y)-40*z
parts=['<svg xmlns="http://www.w3.org/2000/svg" width="900" height="700" viewBox="0 0 900 700">', '<rect width="900" height="700" fill="#f7f9fc"/>', '<g font-family="sans-serif" fill="#15253c"><text x="35" y="38" font-size="24">Bezier surface and flat-cutter envelope</text><text x="35" y="65" font-size="15">20 x 20 mm · cutter diameter 4 mm · fixed axis Z · offline only</text></g>']
def line(ps,color,width=1):
    parts.append('<polyline fill="none" stroke="'+color+'" stroke-width="'+str(width)+'" points="'+' '.join(f'{x:.2f},{y:.2f}' for x,y in ps)+'"/>')
for i in range(21):
    line([project((i,j/2,height(i,j/2))) for j in range(41)],'#b5c1d2')
    line([project((j/2,i,height(j/2,i))) for j in range(41)],'#b5c1d2')
line([project((x,y,height(x,y))) for x,y,z in points],'#347bbc',2)
line([project(p) for p in points],'#be431f',3)
parts += ['<g font-family="sans-serif" font-size="14"><text x="35" y="490" fill="#347bbc">Blue: surface boundary</text><text x="35" y="515" fill="#be431f">Orange: tool-tip centre path, lifted to clear the full disk</text><text x="35" y="550" fill="#15253c">Centre-on-surface would gouge the neighbouring raised material.</text><text x="35" y="578" fill="#15253c">This does not check stock, holder, fixtures or robot collisions.</text><text x="35" y="620" fill="#15253c">z = 8u(1-u) + 4v(1-v); surface maximum = 3 mm</text></g></svg>']
(root/'docs/bezier-crown.svg').write_text('\n'.join(parts))
