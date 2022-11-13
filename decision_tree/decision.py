import numpy as np
import pandas as pd
from sklearn.linear_model import LinearRegression, Ridge, Lasso, ElasticNet
from sklearn import datasets
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.datasets import load_boston
from sklearn.tree import DecisionTreeClassifier, plot_tree
from sklearn.tree import export_graphviz
from sklearn import tree
import graphviz
import pydotplus
from IPython.core.display import Image


data_pd = pd.read_excel(io='D:/ir_data.xlsx',sheet_name='Sheet', usecols='C:F')
x= data_pd[['edge','hist','mean']].values
y= data_pd['in/out'].values

# criterion 의 default = gini
decision_tree = DecisionTreeClassifier(criterion='entropy', random_state=0, max_depth=3)
decision_tree = decision_tree.fit(x,y)


export_graphviz(decision_tree,        # 의사결정나무 모형 대입
                out_file = 'tree.dot',     # file로 변환할 것인가
                feature_names = ['edge','hist','mean'], # feature 이름
                class_names = ['in','out'],   # target 이름
                filled = True,        # 그림에 색상을 넣을것인가
                rounded = True,       # 반올림을 진행할 것인가
                special_characters = True)    # 특수문자를 사용하나

with open('tree.dot') as f:
    dot_graph = f.read()
graphviz.Source(dot_graph)


#plt.figure()
#plot_tree(decision_tree, filled=True)
#plt.show()