if ARGV.length < 2
	exit(1)
end

var = ENV.has_key?(ARGV[0]) ? ENV[ARGV[0]] : ARGV[1]
puts var

exit(0)