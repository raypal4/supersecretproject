from django.contrib import admin
from django.urls import path

from django.views.generic.base import TemplateView
from mapper import views

urlpatterns = [
	path('index/', views.index),
	path('dijskra/', TemplateView.as_view(template_name='dijskra_route.html')),
	path('pdijskra/', TemplateView.as_view(template_name='pdijskra_route.html')),
	path('astar/', TemplateView.as_view(template_name='astar_route.html')),
	path('bus_route/', TemplateView.as_view(template_name='bus_route.html')),
]
