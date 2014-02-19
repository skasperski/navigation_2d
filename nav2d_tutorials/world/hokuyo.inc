define hokuyolaser ranger
(
	sensor
	( 			
		# laser-specific properties
		# factory settings for LMS200	
		range [ 0.0  30.0 ]
		fov 180.0
		samples 721
	)

	# generic model properties
	color "blue"
	size [ 0.07 0.07 0.05 ] # dimensions from LMS200 data sheet	
)



