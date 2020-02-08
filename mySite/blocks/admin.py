from django.contrib import admin

from django.contrib.gis.admin import OSMGeoAdmin
from .models import block
# Register your models here.
@admin.register(block)
class ShopAdmin(OSMGeoAdmin):
    list_display = ('name', 'location')
