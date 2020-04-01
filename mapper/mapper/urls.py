from django.contrib import admin
from django.urls import path

from django.views.generic.base import TemplateView
from mapper import views

urlpatterns = [
	path('index/', views.index),
	path('astar/', TemplateView.as_view(template_name='astar_route.html')),
	path('bus/', TemplateView.as_view(template_name='bus_routing.html')),
	path('lrt/', TemplateView.as_view(template_name='lrt_routing.html')),
	path('punggol/', TemplateView.as_view(template_name='punggol.html')),
]
