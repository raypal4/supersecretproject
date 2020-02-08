from django.db import models
from django.contrib.gis.db import models


# Create your models here.
class block(models.Model):
    name = models.CharField(max_length=100)
    location = models.PointField()
