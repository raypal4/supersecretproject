from django import forms

class AddressForm(forms.Form):
	var_org = forms.CharField(label='Origin Address', max_length=100)
	var_dst = forms.CharField(label='Destination Address', max_length=100)