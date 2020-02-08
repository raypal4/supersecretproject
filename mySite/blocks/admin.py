from django.contrib import admin

from django.contrib.gis.admin import OSMGeoAdmin
from .models import block
from leaflet.admin import LeafletGeoAdmin
# Register your models here.
@admin.register(block)
class ShopAdmin(LeafletGeoAdmin):
    list_display = ('name', 'location')
