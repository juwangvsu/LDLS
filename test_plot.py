#import plotly.plotly as ply
import chart_studio.plotly as ply
from plotly.offline import iplot
import plotly.graph_objs as go
import numpy as np
def plot_points(points, size=1.0, opacity=0.8, color=None, colorscale=None,):
    data = [pointcloud(points, size, opacity, color, colorscale)]
    x = points[:,0]
    y = points[:,1]
    z = points[:,2]
    # xy_lim = np.max(np.abs([np.min(x), np.max(x), np.min(y), np.max(y)]))
    xy_lim = 20
    zmin = -7
    zmax = zmin + 2*xy_lim
    layout = go.Layout(margin=dict(l=0,r=0,b=0,t=0),
                       paper_bgcolor='rgb(55,55,55)',
                       scene=dict(aspectmode='cube',
                                xaxis=dict(title='x', range=[0, 2*xy_lim]),
                                yaxis=dict(title='y', range=[-xy_lim,xy_lim]),
                                zaxis=dict(title='z', range=[zmin,zmax])))
    fig = go.Figure(data=data,layout=layout)

    iplot(fig)
def pointcloud(points, size=1.0, opacity=0.8, color=None, colorscale='Rainbow'):
    x = points[:,0]
    y = points[:,1]
    z = points[:,2]
    return go.Scatter3d(x=x,y=y,z=z, mode='markers',
                         marker=dict(size=size, opacity=opacity,
                                     color=color, colorscale=colorscale),
                         )
print('test plot...')
test_points=np.random.rand(100,3)
plot_points(test_points, size=1.2, opacity=1.0)
