function [print_opts] = get_print_opts()

    print_opts = struct();
    
    print_opts.print_intermediate_results = false;
    print_opts.print_headers = false;
    print_opts.print_total_results = false;
    print_opts.print_acceptable_parameters = true;

end