from django import forms

CHOICES = [
    ('Walk', 'Walk'),
    ('Drive', 'Drive'),
	('Bus', 'Bus'),
    ]

class AddressForm(forms.Form):
	var_org = forms.CharField(label='Origin Address', max_length=100)
	var_dst = forms.CharField(label='Destination Address', max_length=100)
	var_type = forms.ChoiceField(label='Choice of Transport', choices=CHOICES, widget=forms.RadioSelect)