from django.contrib import admin
from django.urls import path

from django.views.generic.base import TemplateView
from mapper import views

urlpatterns = [
	path('index/', views.index),
	path('result/', TemplateView.as_view(template_name='route.html')),
]
